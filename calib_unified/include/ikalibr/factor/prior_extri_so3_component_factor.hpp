// 用于单独约束旋转的某个分量（如Roll或Pitch）
// struct PriorExtriSO3ComponentFactor {
// private:
//     const double SO3_Component_Value;  // 期望的欧拉角值（弧度）
//     const int Component_Index;         // 0=Roll, 1=Pitch, 2=Yaw
//     double _weight;
// public:
//     template <class T>
//     bool operator()(T const *const *sKnots, T *sResiduals) const {
//         Eigen::Map<Sophus::SO3<T> const> const SO3_Sen1ToRef(sKnots[0]);
//         Eigen::Map<Sophus::SO3<T> const> const SO3_Sen2ToRef(sKnots[1]);
//         Sophus::SO3<T> SO3_Sen1ToSen2_Pred = SO3_Sen2ToRef.inverse() * SO3_Sen1ToRef;
        
//         // 转换为欧拉角
//         Eigen::Vector3<T> euler = SO3_Sen1ToSen2_Pred.log();  // 或者根据实际使用的欧拉角顺序
        
//         residuals[0] = T(_weight) * (euler[Component_Index] - T(SO3_Component_Value));
//         return true;
//     }
// };