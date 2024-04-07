#pragma once

struct DeviceSettings {
  double heSetpoint = std::numeric_limits<T>::quiet_NaN();
  double fanSetpoint = std::numeric_limits<T>::quiet_NaN();
  double coilSetpoint = std::numeric_limits<T>::quiet_NaN();
  double heMotorTime = std::numeric_limits<T>::quiet_NaN();
};

struct ButtonState{
  bool left;
  bool right;
}


class CarpetController {
  public:
    CarpetController();

    void saveSettings();

    void setTextColor(Display& tft);

    void adjustHeSetpoint(ButtonState);
    void adjustFanSetpoint(ButtonState);
    void adjustCoilSetpoint(ButtonState);
    void adjustMotorTime(ButtonState);

    double getHeSetpoint() const { return _settings.heSetpoint; }
    double getFanSetpoint() const { return _settings.fanSetpoint; }
    double getCoilSetpoint() const { return _settings.coilSetpoint; }
    double getMotorTime() const { return _settings.heMotorTime; }

  private:

    void adjustSetpoints(ButtonState current, double increment, double& value);

    void readSettings();

    void startMotorCW();
    void startMotorCCW();
    void stopMotor();

    DeviceSettings _settings = {};
    double _input = 0.0, _input2 = 0.0, _output = 0.0;
    PID _pid;
    OneWire _oneWire;
    DallasTemperature _sensors;

    ButtonState _state = {.left=HIGH, .right=HIGH};
};



