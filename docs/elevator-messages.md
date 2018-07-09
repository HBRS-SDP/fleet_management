# ropod elevator requests
Messages sent by the ropod to the resource manager.

## ropod elevator request
```json
  "header": {
    "type": "elevator-cmd" ,
    ...
  },
  "payload": {
    "metamodel": "ropod-elevator-cmd-schema.json",
    "queryId": "03e27e2f-61fd-4092-94d9-8c48365c41e3",
    "command": "CALL_ELEVATOR",
    "startFloor": 1,
    "goalFloor" : 2,
  }
}
```
### reply

```json
  "header": {
    "type": "elevator-cmd-reply" ,
    ...
  },
  "payload": {
    "metamodel": "ropod-elevator-cmd-schema.json",
    "queryId": "03e27e2f-61fd-4092-94d9-8c48365c41e3",
    "querySucess": true
    "elevatorId": 1
    "elevatorWaypoint": "door-1"
  }
}
```

## robot enters elevator

```json
{
  "header": {
    "type": "elevator-cmd" ,
    ...
  },
  "payload": {
    "metamodel": "ropod-elevator-cmd-schema.json",
    "queryId": "03e27e2f-61fd-4092-94d9-8c48365c41e3",
    "command": "ROBOT_FINISHED_ENTERING",
    //"elevatorId": 1,
    //"startFloor": 1 // Optional ?
  }
}
```
### reply
needed ?

## robot exits elevator
```json
{
  "header": {
    "type": "elevator-cmd" ,
    ...
  },
  "payload": {
    "metamodel": "ropod-elevator-cmd-schema.json",
    "queryId": "03e27e2f-61fd-4092-94d9-8c48365c41e3",
    "command": "ROBOT_FINISHED_EXITING",
    //"elevatorId": 1,
    //"goalFloor": 1 // Optional ?
  }
}
```


### reply
needed?


# resource manager elevator calls
