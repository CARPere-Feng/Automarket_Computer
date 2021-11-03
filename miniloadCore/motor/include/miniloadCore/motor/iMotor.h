//
// Created by miniload on 9/1/21.
//

#ifndef MINILOADCORE_MOTOR_INCLUDE_MINILOADCORE_MOTOR_IMOTOR_H_
#define MINILOADCORE_MOTOR_INCLUDE_MINILOADCORE_MOTOR_IMOTOR_H_

namespace MiniloadCore {
namespace Motor {

class iMotor {
 public:
  iMotor() = default;
  ~iMotor() = default;

  virtual int rot2inc(const double& rotation) = 0;

  virtual bool posMove(const double& rotation, const double& rpmm, const double& rpm) = 0;
  virtual bool posMove(const int& inc, const double& rpmm, const double& rpm) = 0;
  virtual bool moveMeters(const double& meters, const double& rpmm, const double& rpm) = 0;
  virtual bool moveMili(const double& mm, const double& rpmm, const double& rpm) = 0;

  virtual bool velMove(const double& rpm) = 0;
  virtual bool velMove(const int& incpm) = 0;
  virtual bool movempm(const double& mpm) = 0;
  virtual bool movemmpm(const double& mmpm) = 0;

  virtual void setLimit(const int& incMin, const int& incMax, const double& rpmMax) = 0;
 private:
};  // class iMotor

} // namespace Motor
} // namespace MiniloadCore

#endif //MINILOADCORE_MOTOR_INCLUDE_MINILOADCORE_MOTOR_IMOTOR_H_
