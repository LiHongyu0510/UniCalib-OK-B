// 用于单独约束平移的某个分量（如Z方向）
// struct PriorExtriPOSComponentFactor {
// private:
//     const double POS_Component_Value;  // 期望的分量值
//     const int Component_Index;         // 分量索引 (0=X, 1=Y, 2=Z)
//     double _weight;
// public:
//     PriorExtriPOSComponentFactor(double value, int index, double weight)
//         : POS_Component_Value(value), Component_Index(index), _weight(weight) {}
//     template <class T>
//     bool operator()(T const *const *sKnots, T *sResiduals) const {
//         Eigen::Map<Eigen::Vector3<T> const> const POS_Sen1InRef(sKnots[0]);
//         Eigen::Map<Sophus::SO3<T> const> const SO3_Sen2ToRef(sKnots[1]);
//         Eigen::Map<Eigen::Vector3<T> const> const POS_Sen2InRef(sKnots[2]);
//         Sophus::SO3<T> SO3_RefToSen2 = SO3_Sen2ToRef.inverse();
//         Eigen::Vector3<T> POS_Sen1InSen2_Pred =
//             SO3_RefToSen2 * POS_Sen1InRef - SO3_RefToSen2 * POS_Sen2InRef;
//         residuals[0] = T(_weight) * (POS_Sen1InSen2_Pred[Component_Index] - T(POS_Component_Value));
//         return true;
//     }
// };