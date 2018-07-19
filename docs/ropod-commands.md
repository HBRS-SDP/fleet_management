# Commands 
* Start
* Stop
* Pause
* Resume 

```json
{
  "header": {
    "type": "CMD",
    "version": "0.2.1",
    "metamodel": "ropod-msg-schema.json",
    "msgId": "0d05d0bc-f1d2-4355-bd88-edf44e2475c8",
    "timestamp": "2017-11-11T11:11:00Z"
  },
  "payload": {
    "metamodel": "ropod-cmd-schema.json",
    "command": "START",
    "id": "1ee2be63-4657-4450-bf80-b6a7157b86f6"
  }
}
```


```json
{
  "header": {
    "type": "CMD",
    "metamodel": "ropod-msg-schema.json",
    "msgId": "a5339f9e-5cc4-454d-a0d3-383163dc7b45"
  },
  "payload": {
    "metamodel": "ropod-cmd-schema.json",
    "command": "STOP"
  }
}
```



```json
{
  "header": {
    "type": "CMD",
    "metamodel": "ropod-msg-schema.json",
    "msgId": "12b24ce8-db2e-4d15-a393-dded5b55e5eb"
  },
  "payload": {
    "metamodel": "ropod-cmd-schema.json",
    "command": "PAUSE"
  }
}
```


```json
{
  "header": {
    "type": "CMD",
    "metamodel": "ropod-msg-schema.json",
    "msgId": "8d5ca928-d086-41b8-b48a-c1ab9f6b9e89"
  },
  "payload": {
    "metamodel": "ropod-cmd-schema.json",
    "command": "RESUME",
    "id": "2ee2be63-4657-4450-bf80-b6a7157b86f6"
  }
}
```



