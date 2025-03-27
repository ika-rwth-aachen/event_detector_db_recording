## Template Rule

This template rule showcases how to implement a recording rule. It will check if a client that is sharing point clouds is is near a specific xy-location. If such a client is found, its latest pose and point cloud will be stored.

### Evaluation Process

- find first clients with point clouds
- check if client is near a specific xy-location

### Parameters

- `rules/event_detector_db_recording_plugin::TemplateRule/`
  - `enabled` [`false`]: whether the rule is enabled
  - `period` [`default_period`]: period (in seconds) between subsequent analysis rule evaluations
  - `parameters/`
    - `x` [`0.0`]: x-origin of observed area
    - `y` [`0.0`]: y-origin of observed area
    - `radius` [`50.0`]: radius of observed area
