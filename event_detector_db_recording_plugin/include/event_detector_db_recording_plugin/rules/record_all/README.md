## RecordAllRule

This rule records all buffered data (since the last evaluation) of all enabled data types to the database.

### Topics, Services, Parameters

#### Subscribed Topics

None.

#### Published Topics

None.

#### Parameters

All parameters are relative to core event detector parameter [`rule_params.<RULE_NAME>.parameters`](https://github.com/ika-rwth-aachen/event_detector?tab=readme-ov-file#parameters).

| Parameter | Type | Default | Description | Options |
| --- | --- | --- | --- | --- |
| `<DATA_TYPE>` | `bool` | `false` | whether to record a specific data type | see third column in [`datatypes.macro`](./event_detector_db_recording_plugin/include/event_detector_db_recording_plugin/datatypes.macro) |
