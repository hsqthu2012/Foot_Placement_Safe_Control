// #include "ocs2_legged_robot/constraint/FootPlacementConstraint.h" // Suqin
#include "ocs2_legged_robot_annotated/constraint/FootPlacementConstraint.h" 

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

FootPlacementConstraint::FootPlacementConstraint(
    const SwitchedModelReferenceManager& referenceManager,
    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
    size_t contactPointIndex)
    : StateInputConstraint(ConstraintOrder::Linear), // Suqin
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex),
      terrainGap_(0.4), // this initialization will fail, no value get, Suqin
      terrainSizeX_(0.2),
      terrainSizeY_(1.0) {
  if (endEffectorKinematicsPtr_->getIds().size() != 1) {
    throw std::runtime_error(
        "[FootPlacementConstraint] this class only accepts a single "
        "end-effector!");
  }
}

bool FootPlacementConstraint::isActive(scalar_t time) const {
  const scalar_t trotGaitWholeCycle = 0.7;
  const scalar_t currentTime = referenceManagerPtr_->getCurrentTime();
  assert(time >= currentTime);
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_] &&
         time <= currentTime + trotGaitWholeCycle;  // only consider one cycle
}

FootPlacementConstraint::FootPlacementConstraint(
    const FootPlacementConstraint& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_) {}

vector_t FootPlacementConstraint::getValue(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation& preComp) const {
  // Step 0 get start (current) time
  const scalar_t trotGaitWholeCycle = 0.7;
  const scalar_t currentTime = referenceManagerPtr_->getCurrentTime();
  assert(time >= currentTime);
  assert(time <= currentTime + trotGaitWholeCycle);

  // Step 1 get middle of the last stance phase
  scalar_t middle =
      referenceManagerPtr_->getGaitSchedule()->getMiddleOfLastStance(time);

  // Step 2 get target state
  const TargetTrajectories& targetTrajectories =
      referenceManagerPtr_->getTargetTrajectories();
  const vector_t stateReference = targetTrajectories.getDesiredState(middle);

  // Step 3 get target end-effector position
  assert(endEffectorKinematicsPtr_->getIds().size() == 1);
  vector3_t pEEReference =
      endEffectorKinematicsPtr_->getPosition(stateReference).front();

  // Step 4 find a closest region (A,b)
  scalar_t xEE = pEEReference(0);
  scalar_t yEE = pEEReference(1);

  const scalar_t terrainGap = 0.4;
  const scalar_t terrainSizeX = 0.2; // Suqin
  const scalar_t terrainSizeY = 0.5; // Suqin
  const scalar_t terrainMargin = 0.02; // Suqin
  scalar_t terrainIndex = std::round(xEE / terrainGap);
//   scalar_t xMin = terrainIndex * terrainGap - terrainSizeX_ / 2;
//   scalar_t XMax = terrainIndex * terrainGap + terrainSizeX_ / 2;
//   scalar_t yMin = 0 - terrainSizeY_ / 2;
//   scalar_t yMax = 0 + terrainSizeY_ / 2; // Suqin
  scalar_t xMin = terrainIndex * terrainGap - (terrainSizeX - terrainMargin) / 2; // Suqin
  scalar_t XMax = terrainIndex * terrainGap + (terrainSizeX - terrainMargin) / 2; // Suqin
  scalar_t yMin = 0 - (terrainSizeY - terrainMargin) / 2; // Suqin
  scalar_t yMax = 0 + (terrainSizeY - terrainMargin) / 2; // Suqin
  Eigen::Matrix<scalar_t, 4, 3> A;
  A << 1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1, 0;
  Eigen::Matrix<scalar_t, 4, 1> b;
  b << -xMin, XMax, -yMin, yMax;

  // Step 5 s = get swing time left (0.35 -> 0)
  scalar_t swingTimeLeft =
      referenceManagerPtr_->getGaitSchedule()->getSwingTimeLeft();
  assert(swingTimeLeft >= 0);
  assert(swingTimeLeft <= trotGaitWholeCycle / 2);
  Eigen::Matrix<scalar_t, 4, 1> s;
//   s << swingTimeLeft, swingTimeLeft, swingTimeLeft, swingTimeLeft; // Suqin
  s << swingTimeLeft, swingTimeLeft, swingTimeLeft, swingTimeLeft; // Suqin

  // Step 6 add constraint Ax + b + s >=0
  vector3_t pEE = endEffectorKinematicsPtr_->getPosition(state).front();

  // Suqin
//   std::cout << "A = " << A << std::endl;
//   std::cout << "pEE = " << pEE << std::endl;
//   std::cout << "b = " << b << std::endl;
//   std::cout << "s = " << s << std::endl;
//   std::cout << "xMin = " << xMin << std::endl;
//   std::cout << "XMax = " << XMax << std::endl;
//   std::cout << "yMin = " << yMin << std::endl;
//   std::cout << "yMax = " << yMax << std::endl;

  return A * pEE + b + s;
}

// VectorFunctionLinearApproximation
// FootPlacementConstraint::getLinearApproximation(
//     scalar_t time, const vector_t& state, const vector_t& input,
//     const PreComputation& preComp) const {
//   throw std::runtime_error(
//       "[FootPlacementConstraint::getLinearApproximation] Linear approximation "
//       "not implemented!");
// } // Suqin
VectorFunctionLinearApproximation
FootPlacementConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation& preComp) const {
  VectorFunctionLinearApproximation linearApproximation =
      VectorFunctionLinearApproximation::Zero(getNumConstraints(time),
                                              state.size(), input.size());
  Eigen::Matrix<scalar_t, 4, 3> A;
  A << 1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1, 0;
  const auto positionApprox =
      endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();

  linearApproximation.f.noalias() =
      getValue(time, state, input, preComp);  // TODO use positionApprox?
  linearApproximation.dfdx.noalias() = A * positionApprox.dfdx;
  return linearApproximation;
} // Suqin

// VectorFunctionQuadraticApproximation
// FootPlacementConstraint::getQuadraticApproximation(
//     scalar_t time, const vector_t& state, const vector_t& input,
//     const PreComputation& preComp) const {
//   // TODO
//   // Step 0 get start (current) time
//   const scalar_t trotGaitWholeCycle = 0.7;
//   const scalar_t currentTime = referenceManagerPtr_->getCurrentTime();
//   assert(time >= currentTime);
//   assert(time <= currentTime + trotGaitWholeCycle);

//   // Step 1 get middle of the last stance phase
//   scalar_t middle =
//       referenceManagerPtr_->getGaitSchedule()->getMiddleOfLastStance(time);

//   // Step 2 get target state
//   const TargetTrajectories& targetTrajectories =
//       referenceManagerPtr_->getTargetTrajectories();
//   const vector_t stateReference = targetTrajectories.getDesiredState(middle);

//   // Step 3 get target end-effector position
//   assert(endEffectorKinematicsPtr_->getIds().size() == 1);
//   vector3_t pEEReference =
//       endEffectorKinematicsPtr_->getPosition(stateReference).front();

//   // Step 4 find a closest region (A,b)
//   scalar_t xEE = pEEReference(0);
//   scalar_t yEE = pEEReference(1);

//   const scalar_t terrainGap = 0.3;
//   scalar_t terrainIndex = std::round(xEE / terrainGap);
//   scalar_t xMin = terrainIndex * terrainGap - terrainSizeX_ / 2;
//   scalar_t XMax = terrainIndex * terrainGap + terrainSizeX_ / 2;
//   scalar_t yMin = 0 - terrainSizeY_ / 2;
//   scalar_t yMax = 0 + terrainSizeY_ / 2;
//   Eigen::Matrix<scalar_t, 4, 3> A;
//   A << 1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1, 0;
//   Eigen::Matrix<scalar_t, 4, 1> b;
//   b << -xMin, XMax, -yMin, yMax;

//   // Step 5 s = get swing time left (0.35 -> 0)
//   scalar_t swingTimeLeft =
//       referenceManagerPtr_->getGaitSchedule()->getSwingTimeLeft();
//   assert(swingTimeLeft >= 0);
//   assert(swingTimeLeft <= trotGaitWholeCycle / 2);
//   Eigen::Matrix<scalar_t, 4, 1> s;
//   s << swingTimeLeft, swingTimeLeft, swingTimeLeft, swingTimeLeft;

//   VectorFunctionQuadraticApproximation positionQuadraticApproximation =
//       endEffectorKinematicsPtr_->getPositionQuadraticApproximation(state)
//           .front();

//   const auto numConstraints = getNumConstraints(time);
//   VectorFunctionQuadraticApproximation quadraticApproximation;
//   quadraticApproximation.setZero(numConstraints, state.rows(), input.rows());
//   quadraticApproximation.f = A * positionQuadraticApproximation.f + b + s;
//   quadraticApproximation.dfdx = A * positionQuadraticApproximation.dfdx;
//   quadraticApproximation.dfdxx.emplace_back(
//       positionQuadraticApproximation.dfdxx[0]);
//   quadraticApproximation.dfdxx.emplace_back(
//       -1 * positionQuadraticApproximation.dfdxx[0]);
//   quadraticApproximation.dfdxx.emplace_back(
//       positionQuadraticApproximation.dfdxx[1]);
//   quadraticApproximation.dfdxx.emplace_back(
//       -1 * positionQuadraticApproximation.dfdxx[1]);

//   return quadraticApproximation;
// }; // Suqin

}  // namespace legged_robot
}  // namespace ocs2