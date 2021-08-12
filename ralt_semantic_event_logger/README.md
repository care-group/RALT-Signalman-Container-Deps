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
        logic:                          // first-order logic description of event
            predicate: some_relation    // custom or set to 'event' for EventName(e)
            x: some_subject             // custom or set to 'none'
            y: some_object              // custom, set to 'none', or set to 'state' for some_relation(sensor state)
        reports: type of report         // 'binary', 'numerical', 'numerical_threshold', or 'energy'
        inverted: true/false            // fixes mismatch in sensor logic and semantic event
        record: all                     // currently, only 'all' semantic events is supported
```

Some example configurations is as follows:
```
---
sensors:
  Sofa Occupied LHS:
    event: SofaEvent
    logic:
      predicate: isOccupied
      x: Sofa
      y: state
    reports: binary
    inverted: true
    record: all
  Bedroom Blinds Control:
    event: BlindsEvent
    logic:
      predicate: isOpen
      x: Blind
      y: state
    reports: numerical_threshold
    threshold: 50
    inverted: false
    record: all
  Kettle Power:
    event: KettleOnEvent
    logic:
      predicate: isOn
      x: Kettle
      y: state
    reports: energy
    threshold: 1200
    inverted: false
    record: all
```

For a full list of examples, see the default ```YAML/sensors.yaml``` file included in this repository.

## Usage

To start the RALT Semantic Event Logger, run: ```python3 main.py```.

A new output CSV file is created automatically in the ```output``` directory (or ```shared/output/CSV``` for Docker), with the label ```events_YYYYMMDD-HHMMSS.csv```.
