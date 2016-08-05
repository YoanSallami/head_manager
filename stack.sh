rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: true, toasterSimuHuman: false, pr2Robot: true, spencerRobot: false, toasterSimuRobot: false, toasterSimuObject: false, arObject : true}" 

rosservice call /database_manager/execute "{command: 'EMPTY', type: 'ALL', facts: [], agent: '', order: '', areaTopic: false ,agentTopic: false ,move3dTopic: false ,pdgTopic: false}"

rosservice call /agent_monitor/monitor_all_agents "monitorAll: true"
rosservice call /agent_monitor/add_joint_to_agent "{jointName: 'head', agentId: 'HERAKLES_HUMAN1'}"
rosservice call /agent_monitor/add_joint_to_agent "{jointName: 'rightHand', agentId: 'HERAKLES_HUMAN1'}"
rosservice call /agent_monitor/add_joint_to_agent "{jointName: 'laser_tilt_mount_link', agentId: 'pr2'}"

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'interaction'
  myOwner: 'pr2'
  areaType: ''
  factType: 'interaction'
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 2.0
  poly:
    points:
    - {x: 0, y: -1, z: 0}
    - {x: 2, y: -2, z: 0}
    - {x: 2, y: 2, z: 0}
    - {x: 0, y: 1, z: 0}
    - {x: 0, y: -1, z: 0}
  insideEntities: [0]"

  rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'type', propertyType: 'state', subProperty: '', subjectId: RED_CUBE'', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'BLACK_CUBE', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'TABLE_4', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'PLACEMAT_RED', targetId: 'AREA', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"
