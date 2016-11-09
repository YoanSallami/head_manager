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
    - {x: 0.4, y: -0.9, z: 0}
    - {x: 0.4, y: 0.9, z: 0}
    - {x: -0.4, y: 0.9, z: 0}
    - {x: -0.4, y: -0.9, z: 0}
  insideEntities: [0]"
