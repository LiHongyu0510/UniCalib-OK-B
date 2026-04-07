// iKalibr: Pangolin-backed Viewer implementation (replaces tiny-viewer).
#include "viewer/viewer_pangolin.h"
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/gl/opengl_render_state.h>
#include <pangolin/handler/handler.h>
#include <set>
#include <thread>

namespace ns_ikalibr {

// ----- MultiViewerPangolin -----
MultiViewerPangolin::MultiViewerPangolin(const ns_viewer::MultiViewerConfigor& config)
    : _config(config) {}

MultiViewerPangolin::~MultiViewerPangolin() {
  _active = false;
  pangolin::QuitAll();
}

std::vector<std::size_t> MultiViewerPangolin::AddEntity(
    const std::vector<ns_viewer::Entity::Ptr>& entities) {
  return AddEntity(entities, "");
}

std::vector<std::size_t> MultiViewerPangolin::AddEntity(
    const std::vector<ns_viewer::Entity::Ptr>& entities, const std::string& view) {
  std::lock_guard<std::mutex> lock(_mutex);
  std::vector<std::size_t> ids;
  auto& list = _store[view];
  auto& handles = _handles[view];
  for (const auto& e : entities) {
    if (!e) continue;
    std::size_t h = _nextHandle++;
    list.push_back(e);
    handles.push_back(h);
    ids.push_back(h);
  }
  return ids;
}

void MultiViewerPangolin::RemoveEntity(const std::vector<std::size_t>& ids,
                                      const std::string& view) {
  if (ids.empty()) return;
  std::lock_guard<std::mutex> lock(_mutex);
  auto itStore = _store.find(view);
  auto itHandles = _handles.find(view);
  if (itStore == _store.end() || itHandles == _handles.end()) return;
  auto& list = itStore->second;
  auto& handles = itHandles->second;
  std::set<std::size_t> toRemove(ids.begin(), ids.end());
  std::vector<ns_viewer::Entity::Ptr> newList;
  std::vector<std::size_t> newHandles;
  for (size_t i = 0; i < handles.size(); ++i) {
    if (toRemove.count(handles[i]) == 0) {
      newList.push_back(list[i]);
      newHandles.push_back(handles[i]);
    }
  }
  list = std::move(newList);
  handles = std::move(newHandles);
}

void MultiViewerPangolin::RemoveEntity(std::size_t id, const std::string& view) {
  RemoveEntity(std::vector<std::size_t>{id}, view);
}

static void DrawEntities(const std::vector<ns_viewer::Entity::Ptr>& entities) {
  for (const auto& e : entities) {
    if (!e) continue;
    if (auto* line = dynamic_cast<ns_viewer::Line*>(e.get())) {
      glColor4f(line->color_.r, line->color_.g, line->color_.b, line->color_.a);
      pangolin::glDrawLine(line->from_(0), line->from_(1), line->from_(2),
                          line->to_(0), line->to_(1), line->to_(2));
    }
  }
}

void MultiViewerPangolin::RunInMultiThread() {
  _active = true;
  std::thread t([this]() {
    pangolin::CreateWindowAndBind(_config.title, _config.window.width, _config.window.height);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    int w = _config.window.width;
    int h = _config.window.height;
    pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(w, h, 420, 420, w / 2.0, h / 2.0, 0.1, 1000);
    pangolin::OpenGlRenderState s_cam(proj, pangolin::ModelViewLookAt(0.5, 0.5, 0.5, 0, 0, 0, pangolin::AxisZ));
    pangolin::Handler3D handler(s_cam);

    pangolin::View& d_multi = pangolin::Display("multi")
        .SetBounds(0.0, 1.0, 0.0, 1.0)
        .SetLayout(pangolin::LayoutEqual);

    pangolin::View& d_sensors = pangolin::Display(Viewer::VIEW_SENSORS)
        .SetAspect(1.0)
        .SetHandler(&handler);
    pangolin::View& d_spline = pangolin::Display(Viewer::VIEW_SPLINE)
        .SetAspect(1.0)
        .SetHandler(&handler);
    pangolin::View& d_map = pangolin::Display(Viewer::VIEW_MAP)
        .SetAspect(1.0)
        .SetHandler(&handler);
    pangolin::View& d_assoc = pangolin::Display(Viewer::VIEW_ASSOCIATION)
        .SetAspect(1.0)
        .SetHandler(&handler);

    d_multi.AddDisplay(d_sensors).AddDisplay(d_spline).AddDisplay(d_map).AddDisplay(d_assoc);

    for (const auto& kv : _config.callBacks) {
      pangolin::RegisterKeyPressCallback(static_cast<int>(kv.first), kv.second);
    }

    while (_active && !pangolin::ShouldQuit()) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      {
        std::lock_guard<std::mutex> lock(_mutex);
        d_sensors.Activate(s_cam);
        DrawEntities(_store[Viewer::VIEW_SENSORS]);
        d_spline.Activate(s_cam);
        DrawEntities(_store[Viewer::VIEW_SPLINE]);
        d_map.Activate(s_cam);
        DrawEntities(_store[Viewer::VIEW_MAP]);
        d_assoc.Activate(s_cam);
        DrawEntities(_store[Viewer::VIEW_ASSOCIATION]);
      }
      pangolin::FinishFrame();
    }
    _active = false;
  });
  t.detach();
}

}  // namespace ns_ikalibr
