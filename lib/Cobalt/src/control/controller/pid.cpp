#include "cobalt/control/controller/pid.hpp"

// ---------------- Getters ----------------
/**
 *  @brief Get proportional gain in the controller.
 *  @return `Kp` Proportional gain.
 */
float cobalt::control::PID::getKp() const { return Kp_; }

/**
 *  @brief Get integral gain in the controller.
 *  @return `Ki` Integral gain.
 */
float cobalt::control::PID::getKi() const { return Ki_; }

/**
 *  @brief Get differential gain in the controller.
 *  @return `Kd` Differential gain.
 */
float cobalt::control::PID::getKd() const { return Kd_; }

/**
 *  @brief Get the differential paths LPF time constant
 *  @return `tau` time constant. [ rad/s ]
 */
float cobalt::control::PID::getTau() const { return tau_; }

/**
 *  @brief Get the discrete-time sampling period.
 *  @return `Ts` Sampling period of the controller. [ s ]
 */
float cobalt::control::PID::getSamplePeriod() const { return Ts_; }

/**
 *  @brief Get the minimum output the controller can output.
 *  @return `outMin` Minimum value controller is configured to be able to output.
 */
float cobalt::control::PID::getOutMin() const { return outMin_; }

/**
 *  @brief Get the maximum output the controller can output.
 *  @return `outMax` Maximum value controller is configured to be able to output.
 */
float cobalt::control::PID::getOutMax() const { return outMax_; }

/**
 *  @brief Get the current proportional output contribution.
 */
float cobalt::control::PID::getPPath() const { return state_.pathP; }

/**
 *  @brief Get the current proportional output contribution.
 */
float cobalt::control::PID::getIPath() const { return state_.pathI; }

/**
 *  @brief Get the current proportional output contribution.
 */
float cobalt::control::PID::getDPath() const { return state_.pathD; }

/**
 *  @brief Get the reference point/value the controller is set to.
 *  @return `ref` Reference value of the controller.
 */
float cobalt::control::PID::getRef() const { return ref_; }

/**
 *  @brief Get the last measurement given to the controller.
 *  @return `measurement` Last measurement the controller got.
 */
float cobalt::control::PID::getMeasure() const { return measure_; }

/**
 *  @brief Get the most recent error signal value.
 *  @return `err` Last error value.
 */
float cobalt::control::PID::getError() const { return (ref_ - measure_); }

/**
 *  @brief Get the last output of the controller.
 *  @return `output` Last output the controller set out.
 */
float cobalt::control::PID::getOutput() const { return output_; }

/**
 *  @brief Get the set name of the controller.
 *  @return `name` Name of controller.
 */
std::string cobalt::control::PID::getName() const { return name_; }


// ---------------- Setters ----------------
/**
 *  @brief Set proportional gain in the controller.
 *  @param Kp Proportional gain.
 * 
 *  @return If the new value was valid or not.
 *  @note The value will not be set if the input is not considered valid.
 */
bool cobalt::control::PID::setKp(float Kp) {
    if(Kp >= 0.0f) {
        Kp_ = Kp;
        return true;
    }
    else {
        return false;
    }
}

/**
 *  @brief Set integral gain in the controller.
 *  @param Ki Integral gain.
 * 
 *  @return If the new value was valid or not.
 *  @note The value will not be set if the input is not considered valid.
 */
bool cobalt::control::PID::setKi(float Ki) {
    if(Ki >= 0.0f) {
        Ki_ = Ki;
        return true;
    }
    else {
        return false;
    }
}

/**
 *  @brief Set differential gain in the controller.
 *  @param Kd Differential gain.
 * 
 *  @return If the new value was valid or not.
 *  @note The value will not be set if the input is not considered valid.
 */
bool cobalt::control::PID::setKd(float Kd) {
    if(Kd >= 0.0f) {
        Kd_ = Kd;
        return true;
    }
    else {
        return false;
    }
}

/**
 *  @brief Set all three gains(Kp, Ki, Kd) in the controller.
 *  @param Kp Proportional gain.
 *  @param Ki Integral gain.
 *  @param Kd Differential gain.
 * 
 *  @return If the new values were valid or not.
 *  @note The value will not be set if an input is not considered valid.
 */
bool cobalt::control::PID::setGains(float Kp, float Ki, float Kd) {
    bool pSuccess = setKp(Kp);
    bool iSuccess = setKi(Ki);
    bool dSuccess = setKd(Kd);

    return ((pSuccess && iSuccess) && dSuccess);
}

/**
 *  @brief Set the differential paths LPF time constant. 
 *  
 *  @param tau LPF time constant.
 *  @return If the new value was valid or not.
 *  @note `tau` should ideally be at least double the sample period. 
 */
bool cobalt::control::PID::setTau(float tau) {
    if(tau >= 2*Ts_) {
        tau_ = tau;
        return true;
    }
    else {
        return false;
    }
}

/**
 *  @brief Set the controller sample period. 
 *  
 *  @param Ts sample period the controller will run at.
 *  @return If the new value was valid or not.
 *  @note `Ts` should ideally be at least 10 time smaller than the plants/ADCs sampling time.
 */
bool cobalt::control::PID::setSampleTime(float tau) {
    if(tau >= 0.0f) {
        tau_ = tau;
        return true;
    }
    else {
        return false;
    }
}

/**
 *  @brief Set the upper and lower limits on the controller output.
 *  
 *  @param min Minimum allowed controller output.
 *  @param max Maximum allowed controller output.
 *  @return If the new value was valid or not.
 *  @note The value will not be set if the input is not considered valid.
 */
bool cobalt::control::PID::setLimits(float min, float max) {
    if(min < max) {
        outMin_ = min;
        outMax_ = max;
        return true;
    }
    else {
        return false;
    }
}

/**
 *  @brief Set the reference/tracking value the controller uses.
 *  
 *  @param ref Reference signal value to track.
 */
bool cobalt::control::PID::setRef(float ref) {
    ref_ = ref;
    return true;
}

/**
 *  @brief Set the internal controller name.
 *  
 *  @param name New controller name.
 */
bool cobalt::control::PID::setName(const std::string &name) {
    name_ = name;
    return true;
}

// ---------------- Functionality ----------------
/**
 *  @brief Reset the memory/state of the controller.
 * 
 *  This will reset the proportional, integral & derivative calculation paths; previous errors & measurements; and last measurement & output down to 0
 *  
 *  @param initialMeasure Set the initial condition/measurement of the output for t = 0. Set this to prevent controller jerks.
 */
bool cobalt::control::PID::reset(float initialMeasure) {
    state_.pathP = 0.0f;
    state_.pathI = 0.0f;
    state_.pathD = 0.0f;

    state_.prevErr = 0.0f;
    state_.prevMeasure = initialMeasure;

    measure_ = 0.0f;
    output_ = 0.0f;

    return true;
}

/**
 *  @brief Update the controller inner state with a new measurement while getting the latest output based on the new measurement
 *  
 *  @param measurement The newest measurement of the system
 *  @return `output` The latest controller output aka. input to the plant
 */
float cobalt::control::PID::update(float measurement) {
    measure_ = measurement;
    float err = ref_ - measure_;    // e[n]


    /* --- Proportional Path --- */
    state_.pathP = Kp_ * err;


    /* --- Integral Path --- */
    state_.pathI += Ki_*(Ts_/2.0f)*(err + state_.prevErr);

        
        // Anti-wind up with clamp
        float integralMin = (state_.pathP > outMin_) ?(outMin_ - state_.pathP) :0.0f;
        float integralMax = (state_.pathP < outMax_) ?(outMax_ - state_.pathP) :0.0f;

        if(state_.pathI < integralMin) { state_.pathI = integralMin; }
        if(state_.pathI > integralMax) { state_.pathI = integralMax; }
        

    /* --- Derivative Path --- */
    state_.pathD = -(Kd_*(2.0f/(2*tau_ + Ts_))*(measure_ - state_.prevMeasure) + ((2*tau_ - Ts_)/(2*tau_ + Ts_))*state_.pathD);


    /* --- Output --- */
    output_ = state_.pathP + state_.pathI + state_.pathD;

        // Output Clamp
        if(output_ < outMin_) { output_ = outMin_; }
        if(output_ > outMax_) { output_ = outMax_; }

    /* --- State Update --- */
    state_.prevErr = err;
    state_.prevMeasure = measure_;

    return output_;
}