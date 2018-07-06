# Task message

```json
{
  "header": {
    "type": "TASK",
    "metamodel": "ropod-msg-schema.json",
    "msgId": "2d05d0bc-f1d2-4355-bd88-edf44e2475c8",
    "timestamp": "2018-04-17T21:27:53Z"
  },
  "payload": {
    "metamodel": "ropod-task-schema.json",
    "taskId": "6ee2be63-4657-4450-bf80-b6a7157b86f6",
    "actions": [
      {
        "actionType": "GOTO",   
        "actionId": "4a15af55-47fa-43e5-ad75-58d138242ab4"
        
      }
    ],
    "teamRobotIds": [
      "3a15af55-47fa-43e5-ad75-58d138242ab4"
    ]

  }
}
```


```json
{
  "header": {
    "type": "TASK",
    "metamodel": "ropod-msg-schema.json",
    "msgId": "2d05d0bc-f1d2-4355-bd88-edf44e2475c8",
    "timestamp": "2018-04-17T21:27:53Z"
  },
  "payload": {
    "metamodel": "ropod-mission-schema.json",
    "taskId": "6ee2be63-4657-4450-bf80-b6a7157b86f6",
    "actions": [
      {
        "actionsType": "GOTO",   
        "actionId": "4a15af55-47fa-43e5-ad75-58d138242ab4", 
        "estimatedDuration": "00:01:52",
        "areas": [
          {
            "name": "c1",    
            "id": "0d19dded-806f-43f0-8777-888de32507fb",
            "debugAreaNodes": [
              {
                    "referenceId": "basement_map",
                    "x": 10,
                    "y": 10,
                    "unit": "m"
              },
              {
                    "referenceId": "basement_map",
                    "x": 10,
                    "y": 20,
                    "unit": "m"
              },
              {
                    "referenceId": "basement_map",
                    "x": 20,
                    "y": 20,
                    "unit": "m"
              },
              {
                    "referenceId": "basement_map",
                    "x": 20,
                    "y": 10,
                    "unit": "m"
              }
            ],
            "debugWaypoint": {
                  "referenceId": "basement_map",
                  "x": 15,
                  "y": 15,
                  "unit": "m"
            }
          },
          {
            "name": "j4",    
            "id": "1d19dded-806f-43f0-8777-888de32507fb"
          }
        ]
      }
    ],
    "teamRobotIds": [
      "3a15af55-47fa-43e5-ad75-58d138242ab4"
    ]

  }
}

```