## rosneuro_integrator_exponential: ExponentialSmoothing Plugin

This package provides the `ExponentialSmoothing` plugin, a simple integrator designed to be loaded by a `rosneuro_integrator` node. Its purpose is to apply a "smoothing factor" (an exponential moving average) to a stream of `NeuroOutput` data.

---

### 1. Plugin Architecture

This plugin class **inherits from the base `rosneuro::Integrator`** interface (or a similar base plugin class).

It is designed to be dynamically loaded by the main `rosneuro_integrator` node. The parent node is responsible for handling all ROS topic subscriptions and publications. This plugin solely implements the core mathematical update logic.

*(Note: This class provides only the smoothing logic. It does **not** perform the multi-topic synchronization found in `GenericIntegrator`.)*

---

### 2. Configuration

To use this plugin, it must be loaded by the `rosneuro_integrator` node, and the smoothing parameter `alpha` must be provided via the command line or a launch file.

* `_plugin:=rosneuro::ExponentialSmoothing` (or the specific C++ class name)
* `_alpha` (float, 0.0 to 1.0): The smoothing factor.
    * A **high `alpha`** (e.g., 0.99) results in slow changes and heavy smoothing (more "memory").
    * A **low `alpha`** (e.g., 0.1) results in
        fast changes and light smoothing (more "responsive").

**Example Launch Command:**
```
rosrun rosneuro_integrator integrator _plugin:=rosneuro::ExponentialSmoothing _alpha:=0.97
```

---

### 3. Core Logic (Formula)

The plugin maintains an internal `current_state`. When the node passes it new `input_data`, it calculates the `new_state` using the following recursive formula:

$$
new\_state = (input\_data \times (1 - \alpha)) + (current\_state \times \alpha)
$$

The calculated `new_state` is then returned to the node to be published, and it also becomes the `current_state` for the next iteration.