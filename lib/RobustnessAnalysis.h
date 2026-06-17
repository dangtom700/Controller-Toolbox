#pragma once

/* Robustness analysis:
- Definition: The uncertainties in plant model itself, sensors and actuators

----(-)-->|sensor|------>|plant model|------->actuator ----------->
     |___________________________________________________|
Fig: Negative feedback model


----(+)-->|sensor|------>|plant model|------->actuator ----------->
     |__________________________________________________|
Fig: Positive feedback model

Things to offer:
- Spawn multiple models off different variances
- Noise the input and output of the plant model to simulate the sensors and actuators
std::vector<model> Monte_Carlo_spawn(num_samples, input_size, output_size, variance_input_vec, variance_output_vec)

*/
#include "random"
#include "algorithm"
#include "PlantModel.h"

// Assuming the plant model is perfect, to study the effect of sensors and actuators
std::vector<ctrl::TransferFunction> spawn_TF_samples(const uint16_t &num_samples,
                                                     const std::vector<float> &numerator_variance,
                                                     const std::vector<float> &denominator_variance)
    std::vector<ctrl::StateSpace> spawn_SS_sample(const uint16_t &num_samples,
                                                  const float &variance,
                                                  const Eigen::MatrixXd &takes) // input or output
