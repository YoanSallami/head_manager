rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: true, toasterSimuHuman: false, pr2Robot: true, spencerRobot: false, toasterSimuRobot: false, toasterSimuObject: false, arObject : true}" 

rosservice call /database_manager/execute "{command: 'EMPTY', type: 'ALL', facts: [], agent: '', order: '', areaTopic: false ,agentTopic: false ,move3dTopic: false ,pdgTopic: false}"

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
  
rosservice call /area_manager/add_area "myArea:
  id: 1
  name: 'action'
  myOwner: 'TABLE_4'
  areaType: ''
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 1.0
  poly:
    points:
    - {x: 0.4, y: -0.8, z: 0}
    - {x: 0.4, y: 0.8, z: 0}
    - {x: -0.6, y: 0.8, z: 0}
    - {x: -0.6, y: -0.8, z: 0}
  insideEntities: [0]"
