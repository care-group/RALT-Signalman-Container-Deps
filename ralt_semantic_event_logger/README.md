# RALT Semantic Event Logger

This software enables semantic logging and broadcast of sensor events from OpenHAB (via REST API) and robots (via ROS).

It is available as a container for RALT Signalman.

## Configuration

You need to provide your OpenHAB ```/rest/items``` endpoint in the ```OPENHAB_URL``` variable of ```main.py```. This must contain authentication credentials if the OpenHAB installation requires it, e.g.:
```
OPENHAB_URL = "https://useremail%40example.com:password@home.myopenhab.org/rest/items"
```

The ```sensors.yaml``` file in the ```YAML``` directory (or ```shared/input/YAML``` for Docker) must be populated with descriptions of the sensors to monitor via OpenHAB.

Each sensor must have an entry as follows:
```
sensors:
    Sensor Label:                       // as it appears in OpenHAB
        event: EventName                // your semantic event name
        alt_predicate_when_true: bool   // 'true' or 'false', if 'true' will use 'alt_predicate', respects inversion
        logic:                          // first-order logic description of event
            predicate: your_pred        // custom or set to 'event' for EventName(e)
            alt_predicate: your_pred    // custom, only required if 'alt_predicate_when_true' is 'true'
            x: some_subject             // custom or set to 'none'
            y: some_object              // custom, set to 'none', or set to 'state' for some_relation(sensor state)
        reports: type of report         // 'binary', 'numerical', 'numerical_threshold', or 'energy'
        inverted: bool                  // 'true' or 'false', fixes mismatch in sensor logic and semantic event
        record: all                     // currently, only 'all' semantic events is supported
        ignore_false: bool              // 'true' or 'false', if 'true' will not record off events, respects inersion
```

Some example configurations is as follows:
```
---
sensors:
  Bathroom Ceiling Light:
    event: ceiling_light_event(e)
    alt_predicate_when_true: false
    logic:
      predicate: alter
      x: ceiling_light
      y: state
    reports: binary
    inverted: false
    record: all
    ignore_false: false
  Bathroom Motion Sensor 1:
    event: presence_event(e)
    alt_predicate_when_true: false
    logic:
      predicate: move
      x: user
      y: bathroom
    reports: binary
    inverted: false
    record: all
    ignore_false: true                  // will not record when the sensor goes bag to idle state
  Kettle Pressure:
    event: kettle_event(e)
    alt_predicate_when_true: true       // since this is set to 'true' this will be a 'take' event
    logic:                              // when kettle is taken off the stand, and a 'give' event
      predicate: take                   // when placed on the stand
      alt_predicate: give
      x: user
      y: kettle
    reports: binary
    inverted: false
    record: all
    ignore_false: false
  Kettle Power:
    event: KettleOnEvent
    logic:
      predicate: isOn
      x: Kettle
      y: state
    reports: energy
    threshold: 1200                     // when threshold exceeded, state will become 'true'
    inverted: false
    record: all
    ignore_false: false
  Kitchen Drawer Top:
    event: kitchen_drawer_event(e)
    alt_predicate_when_true: false
    logic:
      predicate: alter
      x: drawer
      y: state
    reports: binary
    inverted: true                      // the sensor reports 'true' when closed, but we wish
    record: all                         // to record 'true' when open
    ignore_false: false
```

For a full list of examples, see the default ```YAML/sensors.yaml``` file included in this repository.

## Usage

To start the RALT Semantic Event Logger, run: ```python3 main.py```.

A new output CSV file is created automatically in the ```output``` directory (or ```shared/output/CSV``` for Docker), with the label ```events_YYYYMMDD-HHMMSS.csv```.