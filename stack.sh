rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: { frame_id: \"/map\" }, pose: { pose: { position: { x: 4.0, y: 4.0 }, orientation: { x: 0, y: 0, z: -0.0, w: 1.0 } }, covariance: [ 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ] } }'

rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: false, toasterSimuHuman: true, pr2Robot: true, spencerRobot: false, toasterSimuRobot: false, toasterSimuObject: true}" 

rosservice call /database/empty_database

rosservice call /toaster_simu/add_entity "{id: 'TABLE_4', name: 'TABLE_4', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'TABLE_4', ownerId: '', type: 'object', x: 0.8, y: 0.1, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}"

rosservice call /toaster_simu/add_entity "{id: 'RED_CUBE', name: 'RED_CUBE', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'RED_CUBE', ownerId: '', type: 'object', x: 0.6, y: 0.6, z: 0.8, roll: 0.0, pitch: 0.0, yaw: 0.0}"

rosservice call /toaster_simu/add_entity "{id: 'GREEN_CUBE', name: 'GREEN_CUBE', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_CUBE', ownerId: '', type: 'object', x: 0.7, y: 0.7, z: 0.8, roll: 0.0, pitch: 0.0, yaw: 0.0}"

rosservice call /toaster_simu/add_entity "{id: 'BLUE_CUBE', name: 'BLUE_CUBE', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'BLUE_CUBE', ownerId: '', type: 'object', x: 1.1, y: -0.5, z: 0.8, roll: 0.0, pitch: 0.0, yaw: 0.0}"

rosservice call /toaster_simu/add_entity "{id: 'BLACK_CUBE', name: 'BLACK_CUBE', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'BLACK_CUBE', ownerId: '', type: 'object', x: 1.2, y: -0.6, z: 0.8, roll: 0.0, pitch: 0.0, yaw: 0.0}"

rosservice call /toaster_simu/add_entity "{id: 'PLACEMAT_RED', name: 'PLACEMAT_RED', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'PLACEMAT_RED', ownerId: '', type: 'object', x: 0.8, y: 0.1, z: 0.8, roll: 0.0, pitch: 0.0, yaw: 0.0}"

rosservice call /toaster_simu/add_entity "{id: 'HERAKLES_HUMAN1', name: 'HERAKLES_HUMAN1', type: 'human', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'HERAKLES_HUMAN1', ownerId: '', type: 'human', x: 2.0, y: 0.1, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 3.14}"

rosservice call /toaster_simu/add_entity "{id: 'rightHand', name: 'rightHand', type: 'joint', ownerId: 'HERAKLES_HUMAN1'}"
rosservice call /toaster_simu/set_entity_pose "{id: 'rightHand', ownerId: 'HERAKLES_HUMAN1', type: 'joint', x: 2.1, y: 0.45, z: 1.0, roll: 0.0, pitch: 0.0, yaw: 3.14}"

rosservice call /toaster_simu/add_entity "{id: 'base', name: 'base', type: 'joint', ownerId: 'HERAKLES_HUMAN1'}"
rosservice call /toaster_simu/set_entity_pose "{id: 'base', ownerId: 'HERAKLES_HUMAN1', type: 'joint', x: 2.0, y: 0.1, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 3.14}"
