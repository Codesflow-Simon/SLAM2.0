#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
;
using namespace gtsam;
using namespace std;

class BFieldFactor: public NoiseModelFactor1<Pose3> {
  const Vector3 measured_;
  const Rot3 initial_;

public:
  BFieldFactor(Key key, const Vector3& measured, const Rot3& initial,
    const SharedNoiseModel& model) :
    NoiseModelFactor1<Pose3>(model, key), 
    measured_(measured), initial_(initial) {}

  Vector evaluateError(const Pose3 &pose, boost::optional< Matrix & > H=boost::none) const override {
    double scale = norm(measured_);
    Rot3 dir = initial_ * pose.rotation();
    Vector3 hq = scale * dir.matrix() * Vector3(0,1,0);

    if (H) {
      Matrix33 initial_mat = initial_.matrix();
      double h01 = initial_mat.coeff(0,1);
      double h11 = initial_mat.coeff(1,1);
      double h21 = initial_mat.coeff(2,1);

      Matrix33 pose_mat = pose.rotation().matrix();
      double r00 = pose_mat.coeff(0,0);
      double r10 = pose_mat.coeff(1,0);
      double r20 = pose_mat.coeff(2,0);
      double r01 = pose_mat.coeff(0,1);
      double r11 = pose_mat.coeff(1,1);
      double r21 = pose_mat.coeff(2,1);
      double r02 = pose_mat.coeff(0,2);
      double r12 = pose_mat.coeff(1,2);
      double r22 = pose_mat.coeff(2,2);

      *H = (gtsam::Matrix(3,6) << 0, 0, 0, r00*h01, r01*h11, r02*h21, 
                                  0, 0, 0, r10*h01, r11*h11, r12*h21,
                                  0, 0, 0, r20*h01, r21*h11, r22*h21).finished();
    }

    return hq - measured_;
  }
};

// forward declaration of Range functor, assumed partially specified
template <typename A1, typename A2>
struct Range;

template <typename A1, typename A2 = A1, typename T = double>
class RangeFactor : public ExpressionFactorN<T, A1, A2> {
 private:
  typedef RangeFactor<A1, A2> This;
  typedef ExpressionFactorN<T, A1, A2> Base;

 public:
  /// default constructor
  RangeFactor() {}

  RangeFactor(Key key1, Key key2, T measured, const SharedNoiseModel& model)
      : Base({key1, key2}, model, measured) {
    this->initialize(expression({key1, key2}));
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  // Return measurement expression
  Expression<T> expression(const typename Base::ArrayNKeys& keys) const override {
    Expression<A1> a1_(keys[0]);
    Expression<A2> a2_(keys[1]);
    return Expression<T>(Range<A1, A2>(), a1_, a2_);
  }
  
  Vector evaluateError(const A1& a1, const A2& a2,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const
  {
    std::vector<Matrix> Hs(2);
    const auto &keys = Factor::keys();
    const Vector error = Base::unwhitenedError(
      {{keys[0], genericValue(a1)}, {keys[1], genericValue(a2)}}, 
      Hs);
    if (H1) *H1 = Hs[0];
    if (H2) *H2 = Hs[1];
    return error;
  }

  /// print
  void print(const std::string& s = "",
             const KeyFormatter& kf = DefaultKeyFormatter) const override {
    std::cout << s << "RangeFactor" << std::endl;
    Base::print(s, kf);
  }

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Base", boost::serialization::base_object<Base>(*this));
  }
};  // \ RangeFactor