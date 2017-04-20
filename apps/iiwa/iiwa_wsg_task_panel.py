import functools

import director.objectmodel as om
from director import propertyset
from director.tasks import basictasks
from director.tasks.taskuserpanel import TaskUserPanel

import iiwaplanning
import myplanner

class UpdateGraspTargetTask(basictasks.AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty(
            'Position', [0., 0., 0.], attributes=propertyset.PropertyAttributes(
                decimals=3, minimum=-1, maximum=1))
        properties.addProperty(
            'Orientation', [0.0, 0.0, 0.0],
            attributes=propertyset.PropertyAttributes(
                decimals=3, minimum=-360, maximum=360))
        properties.addProperty(
            'Dimensions', [0.1, 0.2, 0.3], attributes=propertyset.PropertyAttributes(
                decimals=3, minimum=0.001, maximum=1))


    def run(self):
        iiwaplanning.setBoxGraspTarget(self.properties.position,
                                       self.properties.orientation,
                                       self.properties.dimensions)
        iiwaplanning.addGraspFrames()


class IiwaWsgTaskPanel(TaskUserPanel):

    rigid_body_target_name = 'Target rigid body'

    def __init__(self, robotSystem, optitrack_vis):
        TaskUserPanel.__init__(self, windowTitle='Task Panel')

        iiwaplanning.init(robotSystem)
        self.planner = myplanner.MyPlanner(robotSystem, self.params)
        self.ui.imageFrame.hide()
        self.robotSystem = robotSystem
        # Robot toy dimensions
        self._default_target_dimensions = [0.06, 0.02, 0.09]
        # Water bottle dimensions
        #self._default_target_dimensions = [0.07, 0.07, 0.22]
        # Squishy ball dimensions
        #
        # TODO(sam.creasey): Why does the squishy ball generate such
        # weird / bound up plans at position 1?
        #self._default_target_dimensions = [0.06, 0.06, 0.06]


        self.addManualButton('add grasp frames', self.addGraspFrameFromList)
        self.addManualButton('plan pregrasp', self.planner.planPreGrasp)
        self.addManualButton('plan grasp', self.planner.planGrasp)

        self.params.addProperty(
            self.rigid_body_target_name, 0,
            attributes=propertyset.PropertyAttributes(enumNames=[""]))
        optitrack_vis.connectRigidBodyListChanged(self.rigidBodyListChanged)

        self.params.addProperty(
            'Frame 1', [0.8, 0.36, 0.30],
            attributes=propertyset.PropertyAttributes(singleStep=0.01))
        self.params.addProperty(
            'Frame 2', [0.8, -0.36, 0.30],
            attributes=propertyset.PropertyAttributes(singleStep=0.01))

        self.addTasks()

    def rigidBodyListChanged(self, body_list):
        old_name = self.params.getPropertyEnumValue(self.rigid_body_target_name)
        print "old selection", old_name
        self.params.setProperty(self.rigid_body_target_name, 0)
        self.params.setPropertyAttribute(self.rigid_body_target_name, 'enumNames',
                                         body_list)
        if old_name in body_list:
            self.params.setProperty(self.rigid_body_target_name, old_name)

    def addGraspFrameFromList(self):
        target_name = self.params.getPropertyEnumValue(self.rigid_body_target_name)
        if len(target_name):
            obj = om.findObjectByName(target_name)
            dims = obj.getProperty('Dimensions')

            # In the frames we're dealing with here, the gripper is
            # facing along the X axis. Left/right/above/etc in the
            # notes below are for a box long dimension aligned with
            # world Y (so if it were directly in front of the arm it
            # would have to be gripped from the side/top).
            grasp_offsets  = [
                # Approach the box from the right (when looking at the arm).
                ((dims[0]/4.0, 0.0, 0.0), (-90, 180, 0)),
                # Same as above, gripper flipper
                ((dims[0]/4.0, 0.0, 0.0), (90, 180, 0)),
                # Attack from below, both gripper orientations.
                ((0.0, dims[1]/4.0, 0.0), (-90, 180, 90)),
                ((0.0, dims[1]/4.0, 0.0), (90, 180, 90)),
                # Approach from above:
                ((0.0, -dims[1]/4.0, 0.0), (-90, 0, 90)),
                ((0.0, -dims[1]/4.0, 0.0), (90, 0, 90)),
                # Approach from the left
                ((-dims[0]/4.0, 0.0, 0.0), (-90, 0, 0)),
                ((-dims[0]/4.0, 0.0, 0.0), (90, 0, 0)),
            ]

            for i, grasp_offset in enumerate(grasp_offsets):
                iiwaplanning.makeGraspFrames(
                    obj, grasp_offset, pregraspOffset=-(dims[0]/2.0 + 0.02),
                    suffix=' %d' % i)
            self.planner.setAffordanceName(target_name)
            self.planner.selectGraspFrameSuffix()

    def addGraspFrameFromProperty(self, name):
        position = self.params.getProperty(name)
        iiwaplanning.setBoxGraspTarget(position,
                                       [0., 0., 0.],
                                       self._default_target_dimensions)
        self.planner.setAffordanceName('box')
        self.planner.addGraspFrames()

    def onPropertyChanged(self, propertySet, propertyName):
        print "property changed", propertyName, propertySet.getProperty(propertyName)

    def addTasks(self):
        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(name, func, parent=None):
            addTask(basictasks.CallbackTask(callback=func, name=name), parent=parent)

        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder

        def addPlanAndExecute(name, planFunc):
            old_folder = self.folder
            addFolder(name, parent=self.folder)
            addFunc(name, planFunc)
            addTask(basictasks.DelayTask(name='wait', delayTime=0.25))
            addFunc('execute', self.planner.commitManipPlan)
            addFunc('wait for execute', self.planner.waitForExecute)
            self.folder = old_folder


        addFolder('pick and place 1->2')
        addFunc('Target Frame 1',
                functools.partial(self.addGraspFrameFromProperty, 'Frame 1'))
        addPlanAndExecute('plan pregrasp', self.planner.planPreGrasp)
        addPlanAndExecute('plan grasp', self.planner.planGrasp)
        addFunc('close gripper', self.planner.closeGripper)
        addTask(basictasks.DelayTask(name='wait', delayTime=1.0))
        addPlanAndExecute('plan prerelease', self.planner.planPreGrasp)
        addFunc('Target Frame 2',
                functools.partial(self.addGraspFrameFromProperty, 'Frame 2'))
        addPlanAndExecute('plan prerelease', self.planner.planPreGrasp)
        addPlanAndExecute('plan release', self.planner.planGrasp)
        addFunc('open gripper', self.planner.openGripper)

        addFolder('pick and place 2->1')
        addFunc('Target Frame 2',
                functools.partial(self.addGraspFrameFromProperty, 'Frame 2'))
        addPlanAndExecute('plan pregrasp', self.planner.planPreGrasp)
        addPlanAndExecute('plan grasp', self.planner.planGrasp)
        addFunc('close gripper', self.planner.closeGripper)
        addTask(basictasks.DelayTask(name='wait', delayTime=1.0))
        addPlanAndExecute('plan prerelease', self.planner.planPreGrasp)
        addFunc('Target Frame 1',
                functools.partial(self.addGraspFrameFromProperty, 'Frame 1'))
        addPlanAndExecute('plan prerelease', self.planner.planPreGrasp)
        addPlanAndExecute('plan release', self.planner.planGrasp)
        addFunc('open gripper', self.planner.openGripper)
