'use client';

import { 
  Terminal, Battery, Cpu, Server, Play, Check, ChevronRight, 
  BookOpen, Layout, Settings, Code, ArrowRight, AlertTriangle, 
  Network, Search, Award, Users, Wrench, Package, FolderTree,
  Lightbulb, Zap, Target, Layers, HelpCircle, Bot, Cog,
  FileCode, GitBranch, Box, Rocket, Radio, Send, Inbox,
  MessageSquare, RefreshCw, Eye, Activity, Filter, Gauge,
  CircuitBoard, Workflow, Combine, BarChart3, Sparkles,
  Sliders, FileJson, Variable, RotateCcw, XCircle, Timer,
  Route, MapPin, Navigation, CheckCircle, ArrowLeftRight
} from 'lucide-react';
import { LectureSidebar, LectureSlide, CodeBlock, TerminalBlock, SlideInfo } from '@/components';

export default function Lecture7() {
  const slides: SlideInfo[] = [
    { id: 'slide-1', title: 'Lecture 7: Actions', icon: BookOpen },
    { id: 'slide-2', title: 'The Problem', icon: HelpCircle },
    { id: 'slide-3', title: 'What is an Action?', icon: Zap },
    { id: 'slide-4', title: 'How Actions Work', icon: Workflow },
    { id: 'slide-5', title: 'When to Use What', icon: Layers },
    { id: 'slide-6', title: 'Action Interface', icon: FileCode },
    { id: 'slide-7', title: 'Building the Interface', icon: Package },
    { id: 'slide-8', title: 'Action Server: Setup', icon: Server },
    { id: 'slide-9', title: 'Goal Callback', icon: Target },
    { id: 'slide-10', title: 'Execute Callback', icon: Play },
    { id: 'slide-11', title: 'Minimal Server Code', icon: Code },
    { id: 'slide-12', title: 'Testing with CLI', icon: Terminal },
    { id: 'slide-13', title: 'Action Client: Setup', icon: Send },
    { id: 'slide-14', title: 'Client Callbacks', icon: Inbox },
    { id: 'slide-15', title: 'Minimal Client Code', icon: Code },
    { id: 'slide-16', title: 'Running Both Together', icon: Activity },
    { id: 'slide-17', title: 'Feedback: Server Side', icon: Radio },
    { id: 'slide-18', title: 'Feedback: Client Side', icon: Eye },
    { id: 'slide-19', title: 'Running with Feedback', icon: Terminal },
    { id: 'slide-20', title: 'Cancel: Server Side', icon: XCircle },
    { id: 'slide-21', title: 'Cancel: Client Side', icon: RotateCcw },
    { id: 'slide-22', title: 'CLI Tools', icon: Wrench },
    { id: 'slide-23', title: 'Summary', icon: Layout },
    { id: 'slide-24', title: 'Homework', icon: Award },
    { id: 'slide-25', title: 'Next Lecture', icon: ArrowRight },
  ];

  // ── Action interface definition ──
  const actionInterfaceCode = `# NavigateToPosition.action

# Goal - where the robot should navigate
float64 target_x
float64 target_y
---
# Result - the final outcome
float64 final_x
float64 final_y
bool success
string message
---
# Feedback - progress during navigation
float64 current_x
float64 current_y
float64 distance_remaining
float64 completion_percentage`;

  // ── CMakeLists addition ──
  const cmakeListsCode = `# my_robot_interfaces/CMakeLists.txt
rosidl_generate_interfaces(\${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/AssignTask.srv"
  "srv/GetTaskStatus.srv"
  "srv/CancelTask.srv"
  "action/NavigateToPosition.action"
)`;

  // ── Server setup snippet ──
  const serverSetupCode = `import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import NavigateToPosition


class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')
        self.current_x = 0.0
        self.current_y = 0.0

        self.action_server = ActionServer(
            self,
            NavigateToPosition,
            'navigate_to_position',
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback
        )

        self.get_logger().info("Navigation Server ready at (0.0, 0.0)")`;

  // ── Goal callback ──
  const goalCallbackCode = `def goal_callback(self, goal_request):
    target_x = goal_request.target_x
    target_y = goal_request.target_y
    self.get_logger().info(
        f"Goal received: navigate to ({target_x}, {target_y})"
    )

    # Validate: coordinates must be within warehouse
    if target_x < 0 or target_y < 0:
        self.get_logger().warn("Rejected: negative coordinates")
        return GoalResponse.REJECT

    if target_x > 100.0 or target_y > 100.0:
        self.get_logger().warn("Rejected: outside warehouse bounds")
        return GoalResponse.REJECT

    return GoalResponse.ACCEPT`;

  // ── Execute callback ──
  const executeCallbackCode = `def execute_callback(self, goal_handle: ServerGoalHandle):
    self.get_logger().info("Executing navigation...")

    target_x = goal_handle.request.target_x
    target_y = goal_handle.request.target_y

    # Calculate distance and steps (~2 m/s movement)
    distance = math.sqrt(
        (target_x - self.current_x) ** 2 +
        (target_y - self.current_y) ** 2
    )
    steps = max(int(distance / 2.0), 1)
    dx = (target_x - self.current_x) / steps
    dy = (target_y - self.current_y) / steps

    # Simulate movement
    for i in range(steps):
        self.current_x += dx
        self.current_y += dy
        self.get_logger().info(
            f"Step {i + 1}/{steps}: "
            f"({self.current_x:.1f}, {self.current_y:.1f})"
        )
        time.sleep(1.0)

    # Reached destination
    self.current_x = target_x
    self.current_y = target_y
    goal_handle.succeed()

    result = NavigateToPosition.Result()
    result.final_x = target_x
    result.final_y = target_y
    result.success = True
    result.message = "Navigation complete!"
    return result`;

  // ── Minimal complete server ──
  const minimalServerCode = `import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import NavigateToPosition


class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')
        self.current_x = 0.0
        self.current_y = 0.0

        self.action_server = ActionServer(
            self,
            NavigateToPosition,
            'navigate_to_position',
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback
        )
        self.get_logger().info("Navigation Server ready at (0.0, 0.0)")

    def goal_callback(self, goal_request):
        target_x = goal_request.target_x
        target_y = goal_request.target_y
        self.get_logger().info(
            f"Goal received: navigate to ({target_x}, {target_y})"
        )
        if target_x < 0 or target_y < 0:
            self.get_logger().warn("Rejected: negative coordinates")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Executing navigation...")
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y

        distance = math.sqrt(
            (target_x - self.current_x) ** 2 +
            (target_y - self.current_y) ** 2
        )
        steps = max(int(distance / 2.0), 1)
        dx = (target_x - self.current_x) / steps
        dy = (target_y - self.current_y) / steps

        for i in range(steps):
            self.current_x += dx
            self.current_y += dy
            self.get_logger().info(
                f"Step {i + 1}/{steps}: "
                f"({self.current_x:.1f}, {self.current_y:.1f})"
            )
            time.sleep(1.0)

        self.current_x = target_x
        self.current_y = target_y
        goal_handle.succeed()

        result = NavigateToPosition.Result()
        result.final_x = target_x
        result.final_y = target_y
        result.success = True
        result.message = "Navigation complete!"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationServer()
    rclpy.spin(node)
    rclpy.shutdown()`;

  // ── Client setup snippet ──
  const clientSetupCode = `import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from my_robot_interfaces.action import NavigateToPosition


class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.action_client = ActionClient(
            self,
            NavigateToPosition,
            'navigate_to_position'
        )

    def send_goal(self, x, y):
        self.action_client.wait_for_server()

        goal = NavigateToPosition.Goal()
        goal.target_x = x
        goal.target_y = y

        self.get_logger().info(f"Sending goal: navigate to ({x}, {y})")

        self.future = self.action_client.send_goal_async(goal)
        self.future.add_done_callback(self.goal_response_callback)`;

  // ── Client callbacks snippet ──
  const clientCallbacksCode = `def goal_response_callback(self, future):
    goal_handle = future.result()

    if not goal_handle.accepted:
        self.get_logger().warn("Goal was REJECTED!")
        return

    self.get_logger().info("Goal ACCEPTED! Waiting for result...")

    self.result_future = goal_handle.get_result_async()
    self.result_future.add_done_callback(self.result_callback)

def result_callback(self, future):
    result = future.result().result
    status = future.result().status

    if status == GoalStatus.STATUS_SUCCEEDED:
        self.get_logger().info(
            f"SUCCESS! Arrived at ({result.final_x}, {result.final_y})"
        )
    else:
        self.get_logger().warn(f"Failed: {result.message}")`;

  // ── Minimal complete client ──
  const minimalClientCode = `import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from my_robot_interfaces.action import NavigateToPosition


class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.action_client = ActionClient(
            self,
            NavigateToPosition,
            'navigate_to_position'
        )

    def send_goal(self, x, y):
        self.action_client.wait_for_server()

        goal = NavigateToPosition.Goal()
        goal.target_x = x
        goal.target_y = y

        self.get_logger().info(f"Sending goal: navigate to ({x}, {y})")

        self.future = self.action_client.send_goal_async(goal)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Goal was REJECTED!")
            return

        self.get_logger().info("Goal ACCEPTED! Waiting for result...")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"SUCCESS! Arrived at ({result.final_x}, {result.final_y})"
            )
        else:
            self.get_logger().warn(f"Failed: {result.message}")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationClient()
    node.send_goal(25.0, 15.0)
    rclpy.spin(node)
    rclpy.shutdown()`;

  // ── Feedback: server-side addition ──
  const feedbackServerCode = `def execute_callback(self, goal_handle: ServerGoalHandle):
    self.get_logger().info("Executing navigation...")

    target_x = goal_handle.request.target_x
    target_y = goal_handle.request.target_y
    feedback = NavigateToPosition.Feedback()  # Create feedback object

    distance = math.sqrt(
        (target_x - self.current_x) ** 2 +
        (target_y - self.current_y) ** 2
    )
    steps = max(int(distance / 2.0), 1)
    dx = (target_x - self.current_x) / steps
    dy = (target_y - self.current_y) / steps

    for i in range(steps):
        self.current_x += dx
        self.current_y += dy

        # Build and publish feedback
        feedback.current_x = self.current_x
        feedback.current_y = self.current_y
        feedback.distance_remaining = math.sqrt(
            (target_x - self.current_x) ** 2 +
            (target_y - self.current_y) ** 2
        )
        feedback.completion_percentage = ((i + 1) / steps) * 100.0
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(
            f"Moving... {feedback.completion_percentage:.0f}% complete"
        )
        time.sleep(1.0)

    # Finish (same as before)
    self.current_x = target_x
    self.current_y = target_y
    goal_handle.succeed()

    result = NavigateToPosition.Result()
    result.final_x = target_x
    result.final_y = target_y
    result.success = True
    result.message = "Navigation complete!"
    return result`;

  // ── Feedback: client-side addition ──
  const feedbackClientCode = `def send_goal(self, x, y):
    self.action_client.wait_for_server()

    goal = NavigateToPosition.Goal()
    goal.target_x = x
    goal.target_y = y

    self.get_logger().info(f"Sending goal: navigate to ({x}, {y})")

    # Pass feedback_callback to receive progress updates
    self.future = self.action_client.send_goal_async(
        goal,
        feedback_callback=self.feedback_callback
    )
    self.future.add_done_callback(self.goal_response_callback)

def feedback_callback(self, feedback_msg):
    fb = feedback_msg.feedback
    self.get_logger().info(
        f"Position: ({fb.current_x:.1f}, {fb.current_y:.1f}) "
        f"| {fb.distance_remaining:.1f}m left "
        f"| {fb.completion_percentage:.0f}%"
    )`;

  // ── Cancel: server-side additions ──
  const cancelServerCode = `from rclpy.action import ActionServer, GoalResponse, CancelResponse

# In __init__, add cancel_callback to ActionServer:
self.action_server = ActionServer(
    self,
    NavigateToPosition,
    'navigate_to_position',
    goal_callback=self.goal_callback,
    cancel_callback=self.cancel_callback,
    execute_callback=self.execute_callback
)

def cancel_callback(self, goal_handle):
    self.get_logger().info("Cancel request received!")
    return CancelResponse.ACCEPT`;

  // ── Cancel check in execute loop ──
  const cancelExecuteCode = `# Inside the for loop in execute_callback:
for i in range(steps):
    # Check for cancellation FIRST
    if goal_handle.is_cancel_requested:
        goal_handle.canceled()
        self.get_logger().info("Navigation CANCELLED!")

        result = NavigateToPosition.Result()
        result.final_x = self.current_x
        result.final_y = self.current_y
        result.success = False
        result.message = "Navigation cancelled by client"
        return result

    # ... rest of movement + feedback code
    self.current_x += dx
    self.current_y += dy
    goal_handle.publish_feedback(feedback)
    time.sleep(1.0)`;

  // ── Cancel: client-side ──
  const cancelClientCode = `class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.action_client = ActionClient(
            self, NavigateToPosition, 'navigate_to_position'
        )
        self.goal_handle = None

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal rejected!")
            return

        self.get_logger().info("Goal accepted!")

        # Cancel after 3 seconds (for demonstration)
        self.cancel_timer = self.create_timer(
            3.0, self.cancel_goal
        )

        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info("Requesting cancellation...")
            self.goal_handle.cancel_goal_async()
        self.cancel_timer.cancel()  # Only cancel once

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation succeeded!")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                f"Cancelled at ({result.final_x:.1f}, {result.final_y:.1f})"
            )
        else:
            self.get_logger().error(f"Failed: {result.message}")`;

  // ── Homework: PickUpPackage interface ──
  const homeworkInterfaceCode = `# PickUpPackage.action

# Goal
string package_id
---
# Result
bool success
string message
---
# Feedback
string current_step
float64 completion_percentage`;

  // ── Homework: server hints ──
  const homeworkServerHintCode = `# Simulate 5 pickup phases, 1 second each:
phases = [
    "Approaching package",
    "Lowering gripper",
    "Gripping package",
    "Lifting package",
    "Package secured"
]

for i, phase in enumerate(phases):
    # Check for cancel...
    # Publish feedback with current_step and percentage...
    time.sleep(1.0)`;

  return (
    <div className="bg-white min-h-screen font-sans selection:bg-zinc-200 selection:text-zinc-900">
      
      <LectureSidebar 
        slides={slides} 
        lectureNumber={7} 
        lectureTitle="Actions" 
      />

      {/* Main Content */}
      <main className="md:ml-72 transition-all duration-300">
        
        {/* Slide 1: Title */}
        <LectureSlide id="slide-1" title="Actions" subtitle="Long-Running Tasks with Feedback" icon={BookOpen}>
          <div className="mt-12 p-8 bg-zinc-50 rounded-2xl border border-zinc-100">
            <div className="grid gap-6 md:grid-cols-3">
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Lecture</h4>
                <p className="font-medium text-zinc-900">7 of 12</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Focus</h4>
                <p className="font-medium text-zinc-900">Third Communication Type</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Goal</h4>
                <p className="font-medium text-zinc-900">Tasks with Progress &amp; Cancellation</p>
              </div>
            </div>
            <div className="mt-8 pt-8 border-t border-zinc-200">
              <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-4">Today&apos;s Agenda</h4>
              <div className="grid md:grid-cols-5 gap-3">
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <Zap className="inline mb-1" size={20} />
                  <div className="text-xs font-medium">Concept</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <FileCode className="inline mb-1" size={20} />
                  <div className="text-xs font-medium">Interface</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <Server className="inline mb-1" size={20} />
                  <div className="text-xs font-medium">Server</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <Send className="inline mb-1" size={20} />
                  <div className="text-xs font-medium">Client</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <Radio className="inline mb-1" size={20} />
                  <div className="text-xs font-medium">Feedback &amp; Cancel</div>
                </div>
              </div>
            </div>
          </div>

          <div className="mt-8 p-4 bg-blue-50 border border-blue-200 rounded-lg">
            <p className="text-blue-800">
              <strong>Note:</strong> Actions are the most advanced ROS 2 communication type. We&apos;ll build up step by step, starting simple and adding features. A live demo will follow.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 2: The Problem */}
        <LectureSlide id="slide-2" title="The Problem" subtitle="When Services Aren't Enough" icon={HelpCircle}>
          <div className="mb-8">
            <p className="text-xl text-zinc-700">
              Recall our task_manager from Lecture 5. When we assign a task, the service responds instantly. But the robot hasn&apos;t actually <strong>moved</strong> yet.
            </p>
          </div>

          <div className="p-6 bg-zinc-900 text-white rounded-xl mb-8 font-mono text-sm">
            <div className="text-zinc-500"># Service call returns immediately</div>
            <div><span className="text-green-400">Client:</span> &quot;Navigate to (25.0, 15.0)&quot;</div>
            <div><span className="text-blue-400">Server:</span> &quot;Task accepted!&quot;  <span className="text-zinc-500"># ... but robot hasn&apos;t moved yet</span></div>
            <div className="mt-2 text-zinc-500"># 30 seconds of movement... client has NO idea what&apos;s happening</div>
            <div><span className="text-blue-400">Server:</span> ???  <span className="text-zinc-500"># No way to report progress</span></div>
          </div>

          <div className="grid md:grid-cols-3 gap-4 mb-8">
            <div className="p-4 bg-red-50 border border-red-200 rounded-lg">
              <h5 className="font-bold text-red-900 mb-2">No Progress Updates</h5>
              <p className="text-red-800 text-sm">The client is blind during execution. Is the robot stuck? Almost there? No way to know.</p>
            </div>
            <div className="p-4 bg-red-50 border border-red-200 rounded-lg">
              <h5 className="font-bold text-red-900 mb-2">No Cancellation</h5>
              <p className="text-red-800 text-sm">Obstacle detected? Can&apos;t tell the robot to stop. You have to wait for it to finish.</p>
            </div>
            <div className="p-4 bg-red-50 border border-red-200 rounded-lg">
              <h5 className="font-bold text-red-900 mb-2">Blocking Problem</h5>
              <p className="text-red-800 text-sm">Services expect quick responses. A 30-second call blocks the server from handling other requests.</p>
            </div>
          </div>

          <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
            <p className="text-green-800">
              <strong>ROS 2 Actions</strong> solve all three problems: progress feedback, cancellation, and proper handling of long-running tasks.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 3: What is an Action? */}
        <LectureSlide id="slide-3" title="What is an Action?" subtitle="The Third Communication Type" icon={Zap}>
          <div className="mb-8 p-6 bg-zinc-900 text-white rounded-xl text-center">
            <p className="text-2xl font-medium">
              An Action is a <span className="text-blue-400">Goal</span> → <span className="text-green-400">Feedback</span> → <span className="text-purple-400">Result</span> communication pattern for tasks that take time.
            </p>
          </div>

          <div className="grid md:grid-cols-3 gap-6 mb-8">
            <div className="p-6 bg-blue-50 border border-blue-200 rounded-xl text-center">
              <Target className="inline text-blue-600 mb-3" size={32} />
              <h4 className="font-bold text-blue-900 mb-2">Goal</h4>
              <p className="text-blue-800 text-sm">What the client wants done.</p>
              <div className="mt-3 p-2 bg-white rounded text-xs font-mono text-blue-700">
                &quot;Navigate to (25.0, 15.0)&quot;
              </div>
            </div>
            <div className="p-6 bg-green-50 border border-green-200 rounded-xl text-center">
              <Activity className="inline text-green-600 mb-3" size={32} />
              <h4 className="font-bold text-green-900 mb-2">Feedback</h4>
              <p className="text-green-800 text-sm">Progress updates during execution.</p>
              <div className="mt-3 p-2 bg-white rounded text-xs font-mono text-green-700">
                &quot;At (12.5, 7.5) — 50% done&quot;
              </div>
            </div>
            <div className="p-6 bg-purple-50 border border-purple-200 rounded-xl text-center">
              <CheckCircle className="inline text-purple-600 mb-3" size={32} />
              <h4 className="font-bold text-purple-900 mb-2">Result</h4>
              <p className="text-purple-800 text-sm">Final outcome when task completes.</p>
              <div className="mt-3 p-2 bg-white rounded text-xs font-mono text-purple-700">
                &quot;Arrived at (25.0, 15.0)!&quot;
              </div>
            </div>
          </div>

          <div className="p-4 bg-zinc-50 rounded-lg">
            <h5 className="font-bold text-zinc-900 mb-2">Key Properties</h5>
            <ul className="text-zinc-700 text-sm space-y-1">
              <li>• An action has a <strong>name</strong> (like topics and services)</li>
              <li>• An action uses an <strong>interface</strong> with three parts: Goal, Result, Feedback</li>
              <li>• There can be only <strong>one server</strong> per action name, but <strong>multiple clients</strong></li>
              <li>• The server can <strong>accept or reject</strong> goals</li>
              <li>• Behind the scenes, actions use a combination of topics and services</li>
            </ul>
          </div>
        </LectureSlide>

        {/* Slide 4: How Actions Work */}
        <LectureSlide id="slide-4" title="How Actions Work" subtitle="Step-by-Step Communication Flow" icon={Workflow}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Here&apos;s the full lifecycle of an action, from sending a goal to receiving the result:
            </p>
          </div>

          <div className="space-y-4 mb-8">
            <div className="flex items-start space-x-4">
              <div className="w-8 h-8 rounded-full bg-blue-500 text-white flex items-center justify-center flex-shrink-0 font-bold text-sm">1</div>
              <div className="p-4 bg-blue-50 rounded-lg flex-1">
                <h5 className="font-bold text-blue-900">Client sends a Goal</h5>
                <p className="text-blue-800 text-sm">The client sends a goal to the server — e.g., &quot;Navigate to (25.0, 15.0)&quot;.</p>
              </div>
            </div>

            <div className="flex items-start space-x-4">
              <div className="w-8 h-8 rounded-full bg-blue-500 text-white flex items-center justify-center flex-shrink-0 font-bold text-sm">2</div>
              <div className="p-4 bg-blue-50 rounded-lg flex-1">
                <h5 className="font-bold text-blue-900">Server accepts or rejects the Goal</h5>
                <p className="text-blue-800 text-sm">The server validates the goal. If accepted, execution begins. If rejected, communication ends.</p>
              </div>
            </div>

            <div className="flex items-start space-x-4">
              <div className="w-8 h-8 rounded-full bg-green-500 text-white flex items-center justify-center flex-shrink-0 font-bold text-sm">3</div>
              <div className="p-4 bg-green-50 rounded-lg flex-1">
                <h5 className="font-bold text-green-900">Server executes and publishes Feedback</h5>
                <p className="text-green-800 text-sm">While working, the server sends periodic updates: current position, distance remaining, percentage complete.</p>
              </div>
            </div>

            <div className="flex items-start space-x-4">
              <div className="w-8 h-8 rounded-full bg-amber-500 text-white flex items-center justify-center flex-shrink-0 font-bold text-sm">4</div>
              <div className="p-4 bg-amber-50 rounded-lg flex-1">
                <h5 className="font-bold text-amber-900">Client can cancel (optional)</h5>
                <p className="text-amber-800 text-sm">At any point during execution, the client can request cancellation. The server decides whether to accept it.</p>
              </div>
            </div>

            <div className="flex items-start space-x-4">
              <div className="w-8 h-8 rounded-full bg-purple-500 text-white flex items-center justify-center flex-shrink-0 font-bold text-sm">5</div>
              <div className="p-4 bg-purple-50 rounded-lg flex-1">
                <h5 className="font-bold text-purple-900">Server returns the Result</h5>
                <p className="text-purple-800 text-sm">When done (succeeded, failed, or cancelled), the server sends the final result to the client.</p>
              </div>
            </div>
          </div>

          <div className="p-4 bg-zinc-900 text-white rounded-xl text-center text-sm">
            <span className="text-zinc-400">Behind the scenes: Actions use </span>
            <span className="text-red-400">3 services</span>
            <span className="text-zinc-400"> (send goal, cancel, get result) + </span>
            <span className="text-green-400">2 topics</span>
            <span className="text-zinc-400"> (feedback, status)</span>
          </div>
        </LectureSlide>

        {/* Slide 5: When to Use What */}
        <LectureSlide id="slide-5" title="When to Use What" subtitle="Topics vs Services vs Actions" icon={Layers}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Now you know all three ROS 2 communication types. Here&apos;s when to use each:
            </p>
          </div>

          <div className="overflow-x-auto mb-8">
            <table className="w-full text-sm">
              <thead>
                <tr className="bg-zinc-900 text-white">
                  <th className="p-3 text-left rounded-tl-lg">Feature</th>
                  <th className="p-3 text-left">Topics</th>
                  <th className="p-3 text-left">Services</th>
                  <th className="p-3 text-left rounded-tr-lg">Actions</th>
                </tr>
              </thead>
              <tbody>
                <tr className="border-b border-zinc-100">
                  <td className="p-3 font-medium">Pattern</td>
                  <td className="p-3">Publish/Subscribe</td>
                  <td className="p-3">Request/Response</td>
                  <td className="p-3">Goal/Feedback/Result</td>
                </tr>
                <tr className="border-b border-zinc-100 bg-zinc-50">
                  <td className="p-3 font-medium">Duration</td>
                  <td className="p-3">Continuous stream</td>
                  <td className="p-3">Quick (milliseconds)</td>
                  <td className="p-3">Long-running (seconds+)</td>
                </tr>
                <tr className="border-b border-zinc-100">
                  <td className="p-3 font-medium">Feedback</td>
                  <td className="p-3">N/A (always streaming)</td>
                  <td className="p-3">None</td>
                  <td className="p-3">Yes, during execution</td>
                </tr>
                <tr className="border-b border-zinc-100 bg-zinc-50">
                  <td className="p-3 font-medium">Cancellable</td>
                  <td className="p-3">N/A</td>
                  <td className="p-3">No</td>
                  <td className="p-3">Yes</td>
                </tr>
                <tr>
                  <td className="p-3 font-medium">Example</td>
                  <td className="p-3">Sensor data, velocity</td>
                  <td className="p-3">Assign task, get status</td>
                  <td className="p-3">Navigate, pick up package</td>
                </tr>
              </tbody>
            </table>
          </div>

          <div className="grid md:grid-cols-3 gap-4">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg text-center">
              <h5 className="font-bold text-blue-900 mb-1">Topics</h5>
              <p className="text-blue-800 text-xs">&quot;Here&apos;s data, continuously&quot;</p>
            </div>
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg text-center">
              <h5 className="font-bold text-green-900 mb-1">Services</h5>
              <p className="text-green-800 text-xs">&quot;Do this quick thing, tell me the result&quot;</p>
            </div>
            <div className="p-4 bg-purple-50 border border-purple-200 rounded-lg text-center">
              <h5 className="font-bold text-purple-900 mb-1">Actions</h5>
              <p className="text-purple-800 text-xs">&quot;Do this long thing, keep me updated&quot;</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 6: Action Interface */}
        <LectureSlide id="slide-6" title="Defining the Action Interface" subtitle="NavigateToPosition.action" icon={FileCode}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Like services, actions need a custom interface. It has <strong>three sections</strong> separated by <code className="bg-zinc-100 px-2 py-1 rounded">---</code>: Goal, Result, and Feedback.
            </p>
          </div>

          <CodeBlock 
            filename="my_robot_interfaces/action/NavigateToPosition.action"
            code={actionInterfaceCode}
          />

          <div className="mt-6 grid md:grid-cols-3 gap-4">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h5 className="font-bold text-blue-900 mb-2">Goal (top)</h5>
              <p className="text-blue-800 text-xs">What the client sends: target coordinates.</p>
            </div>
            <div className="p-4 bg-purple-50 border border-purple-200 rounded-lg">
              <h5 className="font-bold text-purple-900 mb-2">Result (middle)</h5>
              <p className="text-purple-800 text-xs">Final outcome: where robot ended up, success/failure.</p>
            </div>
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h5 className="font-bold text-green-900 mb-2">Feedback (bottom)</h5>
              <p className="text-green-800 text-xs">Progress updates: current position, distance, percentage.</p>
            </div>
          </div>

          <div className="mt-6 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <h5 className="font-bold text-amber-900 mb-2">Order Matters!</h5>
            <p className="text-amber-800 text-sm">
              It&apos;s always Goal → Result → Feedback. A common mistake is putting Feedback before Result. The file extension is <code className="bg-white px-1 rounded">.action</code> and the name should use UpperCamelCase with a verb (e.g., <code className="bg-white px-1 rounded">NavigateToPosition</code>, <code className="bg-white px-1 rounded">PickUpPackage</code>).
            </p>
          </div>
        </LectureSlide>

        {/* Slide 7: Building the Interface */}
        <LectureSlide id="slide-7" title="Building the Interface" subtitle="Package Setup and Build" icon={Package}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Add the action file to <code className="bg-zinc-100 px-2 py-1 rounded">my_robot_interfaces</code> — the same package where we keep our .msg and .srv files.
            </p>
          </div>

          <div className="space-y-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">1. Create the action folder and file</h4>
              <TerminalBlock 
                command="mkdir -p ~/ros2_ws/src/my_robot_interfaces/action"
                output=""
                title="Terminal"
              />
              <p className="text-zinc-600 text-sm mt-2">Place <code className="bg-zinc-100 px-1 rounded">NavigateToPosition.action</code> in this folder.</p>
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">2. Update CMakeLists.txt</h4>
              <CodeBlock 
                filename="CMakeLists.txt"
                code={cmakeListsCode}
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">3. Build and verify</h4>
              <TerminalBlock 
                command="cd ~/ros2_ws && colcon build --packages-select my_robot_interfaces"
                output={`Starting >>> my_robot_interfaces
Finished <<< my_robot_interfaces [8.5s]

Summary: 1 package finished`}
                title="Terminal"
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">4. Source and check</h4>
              <TerminalBlock 
                command="source install/setup.bash && ros2 interface show my_robot_interfaces/action/NavigateToPosition"
                output={`float64 target_x
float64 target_y
---
float64 final_x
float64 final_y
bool success
string message
---
float64 current_x
float64 current_y
float64 distance_remaining
float64 completion_percentage`}
                title="Terminal"
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 8: Action Server Setup */}
        <LectureSlide id="slide-8" title="Action Server: Setup" subtitle="Creating the Navigation Server Node" icon={Server}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              The action server receives goals, executes them, and returns results. Let&apos;s set up the node:
            </p>
          </div>

          <CodeBlock 
            filename="navigation_server.py"
            code={serverSetupCode}
          />

          <div className="mt-6 grid md:grid-cols-2 gap-4">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h5 className="font-bold text-blue-900 mb-2">Key Imports</h5>
              <ul className="text-blue-800 text-sm space-y-1">
                <li><code className="bg-white px-1 rounded">ActionServer</code> — creates the server</li>
                <li><code className="bg-white px-1 rounded">GoalResponse</code> — accept/reject goals</li>
                <li><code className="bg-white px-1 rounded">ServerGoalHandle</code> — manage active goals</li>
              </ul>
            </div>
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h5 className="font-bold text-green-900 mb-2">ActionServer Arguments</h5>
              <ul className="text-green-800 text-sm space-y-1">
                <li><code className="bg-white px-1 rounded">self</code> — the node</li>
                <li><code className="bg-white px-1 rounded">NavigateToPosition</code> — the interface</li>
                <li><code className="bg-white px-1 rounded">&apos;navigate_to_position&apos;</code> — action name</li>
                <li>Two callbacks: goal + execute</li>
              </ul>
            </div>
          </div>

          <div className="mt-4 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <p className="text-amber-800 text-sm">
              <strong>Note:</strong> Unlike topics and services where you call <code className="bg-white px-1 rounded">self.create_...</code>, the ActionServer constructor takes <code className="bg-white px-1 rounded">self</code> as the first argument.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 9: Goal Callback */}
        <LectureSlide id="slide-9" title="Goal Callback" subtitle="Accept or Reject Incoming Goals" icon={Target}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              When a goal arrives, the goal callback decides whether to accept or reject it. This is your validation step.
            </p>
          </div>

          <CodeBlock 
            filename="navigation_server.py"
            code={goalCallbackCode}
          />

          <div className="mt-6 grid md:grid-cols-2 gap-4">
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <div className="flex items-center mb-2">
                <CheckCircle className="text-green-600 mr-2" size={20} />
                <h5 className="font-bold text-green-900">GoalResponse.ACCEPT</h5>
              </div>
              <p className="text-green-800 text-sm">Goal is valid → proceed to execute_callback.</p>
            </div>
            <div className="p-4 bg-red-50 border border-red-200 rounded-lg">
              <div className="flex items-center mb-2">
                <XCircle className="text-red-600 mr-2" size={20} />
                <h5 className="font-bold text-red-900">GoalResponse.REJECT</h5>
              </div>
              <p className="text-red-800 text-sm">Goal is invalid → client is notified, nothing more happens.</p>
            </div>
          </div>

          <div className="mt-4 p-4 bg-zinc-50 rounded-lg">
            <h5 className="font-bold text-zinc-900 mb-2">What to Validate</h5>
            <ul className="text-zinc-700 text-sm space-y-1">
              <li>• Are coordinates within warehouse bounds?</li>
              <li>• Is the data in the correct range?</li>
              <li>• Is the server already busy with another goal?</li>
            </ul>
          </div>
        </LectureSlide>

        {/* Slide 10: Execute Callback */}
        <LectureSlide id="slide-10" title="Execute Callback" subtitle="Where the Work Happens" icon={Play}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              If the goal is accepted, the execute callback is called. This is where you do the actual work.
            </p>
          </div>

          <CodeBlock 
            filename="navigation_server.py"
            code={executeCallbackCode}
          />

          <div className="mt-6 grid md:grid-cols-3 gap-4">
            <div className="p-3 bg-blue-50 rounded-lg text-center">
              <div className="text-sm font-bold text-blue-900">1. Extract goal data</div>
              <p className="text-blue-700 text-xs mt-1">target_x, target_y from goal_handle.request</p>
            </div>
            <div className="p-3 bg-green-50 rounded-lg text-center">
              <div className="text-sm font-bold text-green-900">2. Do the work</div>
              <p className="text-green-700 text-xs mt-1">Simulate movement with a loop + time.sleep()</p>
            </div>
            <div className="p-3 bg-purple-50 rounded-lg text-center">
              <div className="text-sm font-bold text-purple-900">3. Set state &amp; return result</div>
              <p className="text-purple-700 text-xs mt-1">goal_handle.succeed() + return result</p>
            </div>
          </div>

          <div className="mt-4 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <h5 className="font-bold text-amber-900 mb-2">Goal Final States</h5>
            <div className="grid md:grid-cols-3 gap-3 text-sm">
              <div><code className="bg-white px-1 rounded text-green-700">goal_handle.succeed()</code> — completed successfully</div>
              <div><code className="bg-white px-1 rounded text-red-700">goal_handle.abort()</code> — failed during execution</div>
              <div><code className="bg-white px-1 rounded text-amber-700">goal_handle.canceled()</code> — cancelled by client</div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 11: Minimal Server Code */}
        <LectureSlide id="slide-11" title="Minimal Server Code" subtitle="Complete Working Server" icon={Code}>
          <div className="mb-4">
            <p className="text-lg text-zinc-700">
              Here&apos;s the full minimal navigation server — no feedback or cancellation yet, just the basics:
            </p>
          </div>

          <CodeBlock 
            filename="navigation_server.py (minimal)"
            code={minimalServerCode}
          />

          <div className="mt-4 p-4 bg-green-50 border border-green-200 rounded-lg">
            <p className="text-green-800 text-sm">
              <strong>This server works!</strong> It accepts goals with valid coordinates, simulates movement at ~2 m/s, and returns the final position. We&apos;ll add feedback and cancellation soon.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 12: Testing with CLI */}
        <LectureSlide id="slide-12" title="Testing with CLI" subtitle="Send Goals from the Command Line" icon={Terminal}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Before writing a client node, test the server using <code className="bg-zinc-100 px-2 py-1 rounded">ros2 action send_goal</code>:
            </p>
          </div>

          <div className="space-y-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 1: Start the server</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg navigation_server"
                output={`[INFO] [navigation_server]: Navigation Server ready at (0.0, 0.0)`}
                title="Terminal 1"
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 2: Send a goal</h4>
              <TerminalBlock 
                command={`ros2 action send_goal /navigate_to_position my_robot_interfaces/action/NavigateToPosition "{target_x: 10.0, target_y: 6.0}"`}
                output={`Waiting for an action server to become available...
Sending goal:
  target_x: 10.0
  target_y: 6.0

Goal accepted with ID: a1b2c3d4...
Result:
  final_x: 10.0
  final_y: 6.0
  success: true
  message: Navigation complete!
Goal finished with status: SUCCEEDED`}
                title="Terminal 2"
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Server logs during execution</h4>
              <TerminalBlock 
                command=""
                output={`[INFO] [navigation_server]: Goal received: navigate to (10.0, 6.0)
[INFO] [navigation_server]: Executing navigation...
[INFO] [navigation_server]: Step 1/5: (2.0, 1.2)
[INFO] [navigation_server]: Step 2/5: (4.0, 2.4)
[INFO] [navigation_server]: Step 3/5: (6.0, 3.6)
[INFO] [navigation_server]: Step 4/5: (8.0, 4.8)
[INFO] [navigation_server]: Step 5/5: (10.0, 6.0)`}
                title="Terminal 1 (server output)"
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 13: Action Client Setup */}
        <LectureSlide id="slide-13" title="Action Client: Setup" subtitle="Creating the Client Node" icon={Send}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              The action client sends goals and handles the asynchronous response. Everything is callback-based.
            </p>
          </div>

          <CodeBlock 
            filename="navigation_client.py"
            code={clientSetupCode}
          />

          <div className="mt-6 p-6 bg-zinc-50 rounded-xl">
            <h5 className="font-bold text-zinc-900 mb-4">Key Points</h5>
            <div className="space-y-3 text-sm">
              <div className="flex items-start space-x-3">
                <div className="w-6 h-6 rounded-full bg-blue-500 text-white flex items-center justify-center flex-shrink-0 text-xs font-bold">1</div>
                <p className="text-zinc-700"><code className="bg-white px-1 rounded">ActionClient(self, Interface, name)</code> — creates the client, same pattern as ActionServer</p>
              </div>
              <div className="flex items-start space-x-3">
                <div className="w-6 h-6 rounded-full bg-blue-500 text-white flex items-center justify-center flex-shrink-0 text-xs font-bold">2</div>
                <p className="text-zinc-700"><code className="bg-white px-1 rounded">wait_for_server()</code> — blocks until the server is available (like services)</p>
              </div>
              <div className="flex items-start space-x-3">
                <div className="w-6 h-6 rounded-full bg-blue-500 text-white flex items-center justify-center flex-shrink-0 text-xs font-bold">3</div>
                <p className="text-zinc-700"><code className="bg-white px-1 rounded">send_goal_async(goal)</code> — sends the goal, returns a future. Non-blocking.</p>
              </div>
              <div className="flex items-start space-x-3">
                <div className="w-6 h-6 rounded-full bg-blue-500 text-white flex items-center justify-center flex-shrink-0 text-xs font-bold">4</div>
                <p className="text-zinc-700"><code className="bg-white px-1 rounded">add_done_callback()</code> — register a function to call when the server responds</p>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 14: Client Callbacks */}
        <LectureSlide id="slide-14" title="Client Callbacks" subtitle="Handling Responses and Results" icon={Inbox}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              The client needs two callbacks: one for the goal response (accepted/rejected) and one for the final result.
            </p>
          </div>

          <CodeBlock 
            filename="navigation_client.py"
            code={clientCallbacksCode}
          />

          <div className="mt-6 p-6 bg-zinc-900 text-white rounded-xl">
            <h5 className="font-bold mb-4">Callback Chain</h5>
            <div className="flex items-center justify-center space-x-3 text-sm flex-wrap gap-y-2">
              <div className="p-2 bg-blue-600 rounded">send_goal_async()</div>
              <ArrowRight size={16} />
              <div className="p-2 bg-green-600 rounded">goal_response_callback</div>
              <ArrowRight size={16} />
              <div className="p-2 bg-purple-600 rounded">result_callback</div>
            </div>
            <p className="text-zinc-400 text-xs mt-4 text-center">
              Each callback triggers the next step in the chain. If the goal is rejected, the chain stops.
            </p>
          </div>

          <div className="mt-4 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <p className="text-amber-800 text-sm">
              <strong>Why GoalStatus?</strong> Import <code className="bg-white px-1 rounded">GoalStatus</code> from <code className="bg-white px-1 rounded">action_msgs.msg</code> to check if the goal succeeded, was cancelled, or aborted. This gives you more detail than just checking <code className="bg-white px-1 rounded">result.success</code>.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 15: Minimal Client Code */}
        <LectureSlide id="slide-15" title="Minimal Client Code" subtitle="Complete Working Client" icon={Code}>
          <div className="mb-4">
            <p className="text-lg text-zinc-700">
              Here&apos;s the full minimal client — sends a goal and waits for the result:
            </p>
          </div>

          <CodeBlock 
            filename="navigation_client.py (minimal)"
            code={minimalClientCode}
          />

          <div className="mt-4 p-4 bg-zinc-50 rounded-lg">
            <h5 className="font-bold text-zinc-900 mb-2">In main()</h5>
            <p className="text-zinc-700 text-sm">
              We create the node, send a goal to (25.0, 15.0), then spin the node so callbacks can fire. The send is non-blocking — <code className="bg-white px-1 rounded">rclpy.spin()</code> processes the callbacks as they arrive.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 16: Running Both Together */}
        <LectureSlide id="slide-16" title="Running Both Together" subtitle="Server + Client in Action" icon={Activity}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Start the server in one terminal, the client in another:
            </p>
          </div>

          <div className="space-y-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 1: Server</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg navigation_server"
                output={`[INFO] [navigation_server]: Navigation Server ready at (0.0, 0.0)
[INFO] [navigation_server]: Goal received: navigate to (25.0, 15.0)
[INFO] [navigation_server]: Executing navigation...
[INFO] [navigation_server]: Step 1/14: (1.7, 1.0)
[INFO] [navigation_server]: Step 2/14: (3.4, 2.1)
...
[INFO] [navigation_server]: Step 14/14: (25.0, 15.0)`}
                title="Terminal 1"
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 2: Client</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg navigation_client"
                output={`[INFO] [navigation_client]: Sending goal: navigate to (25.0, 15.0)
[INFO] [navigation_client]: Goal ACCEPTED! Waiting for result...
[INFO] [navigation_client]: SUCCESS! Arrived at (25.0, 15.0)`}
                title="Terminal 2"
              />
            </div>
          </div>

          <div className="mt-6 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <p className="text-amber-800 text-sm">
              <strong>Notice the gap:</strong> The client says &quot;Waiting for result&quot; and then goes silent for ~14 seconds until the server finishes. The client has no idea what&apos;s happening. Let&apos;s fix that with feedback.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 17: Feedback Server Side */}
        <LectureSlide id="slide-17" title="Feedback: Server Side" subtitle="Publishing Progress During Execution" icon={Radio}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              To send feedback, create a Feedback object and call <code className="bg-zinc-100 px-2 py-1 rounded">goal_handle.publish_feedback()</code> inside the execute loop:
            </p>
          </div>

          <CodeBlock 
            filename="navigation_server.py (with feedback)"
            code={feedbackServerCode}
          />

          <div className="mt-6 grid md:grid-cols-2 gap-4">
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h5 className="font-bold text-green-900 mb-2">What Changed</h5>
              <ul className="text-green-800 text-sm space-y-1">
                <li>+ Created <code className="bg-white px-1 rounded">NavigateToPosition.Feedback()</code></li>
                <li>+ Fill in current position + distance + percentage</li>
                <li>+ Call <code className="bg-white px-1 rounded">goal_handle.publish_feedback(feedback)</code></li>
              </ul>
            </div>
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h5 className="font-bold text-blue-900 mb-2">When to Publish</h5>
              <ul className="text-blue-800 text-sm space-y-1">
                <li>After each movement step</li>
                <li>At regular intervals during execution</li>
                <li>Don&apos;t publish too frequently (flooding)</li>
              </ul>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 18: Feedback Client Side */}
        <LectureSlide id="slide-18" title="Feedback: Client Side" subtitle="Receiving Progress Updates" icon={Eye}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              On the client, add a <code className="bg-zinc-100 px-2 py-1 rounded">feedback_callback</code> when sending the goal:
            </p>
          </div>

          <CodeBlock 
            filename="navigation_client.py (with feedback)"
            code={feedbackClientCode}
          />

          <div className="mt-6 p-6 bg-zinc-900 text-white rounded-xl">
            <h5 className="font-bold mb-3">Two Changes:</h5>
            <div className="space-y-3 text-sm">
              <div>
                <span className="text-green-400">1.</span> Pass <code className="text-blue-400">feedback_callback=self.feedback_callback</code> to <code className="text-blue-400">send_goal_async()</code>
              </div>
              <div>
                <span className="text-green-400">2.</span> Implement the callback — access feedback via <code className="text-blue-400">feedback_msg.feedback</code>
              </div>
            </div>
          </div>

          <div className="mt-4 p-4 bg-zinc-50 rounded-lg">
            <p className="text-zinc-700 text-sm">
              <strong>Note:</strong> The feedback_callback parameter is optional. If you don&apos;t need feedback, just don&apos;t pass it. The server will still publish feedback, but the client won&apos;t receive it.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 19: Running with Feedback */}
        <LectureSlide id="slide-19" title="Running with Feedback" subtitle="Full Terminal Output" icon={Terminal}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Now the client receives real-time progress updates during navigation:
            </p>
          </div>

          <div className="space-y-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Client output (with feedback)</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg navigation_client"
                output={`[INFO] [navigation_client]: Sending goal: navigate to (25.0, 15.0)
[INFO] [navigation_client]: Goal ACCEPTED! Waiting for result...
[INFO] [navigation_client]: Position: (1.7, 1.0) | 27.2m left | 7%
[INFO] [navigation_client]: Position: (3.4, 2.1) | 25.1m left | 14%
[INFO] [navigation_client]: Position: (5.1, 3.1) | 22.9m left | 21%
[INFO] [navigation_client]: Position: (8.6, 5.1) | 18.6m left | 36%
...
[INFO] [navigation_client]: Position: (23.3, 14.0) | 2.1m left | 93%
[INFO] [navigation_client]: Position: (25.0, 15.0) | 0.0m left | 100%
[INFO] [navigation_client]: SUCCESS! Arrived at (25.0, 15.0)`}
                title="Terminal"
              />
            </div>
          </div>

          <div className="mt-6 p-4 bg-green-50 border border-green-200 rounded-lg">
            <h5 className="font-bold text-green-900 mb-2">Now the client knows:</h5>
            <div className="grid md:grid-cols-3 gap-3 text-sm">
              <div className="p-2 bg-white rounded text-center text-green-800">Where the robot is</div>
              <div className="p-2 bg-white rounded text-center text-green-800">How far it has to go</div>
              <div className="p-2 bg-white rounded text-center text-green-800">Completion percentage</div>
            </div>
          </div>

          <div className="mt-4 p-4 bg-zinc-50 rounded-lg">
            <p className="text-zinc-700 text-sm">
              You can also see feedback from CLI by adding <code className="bg-white px-1 rounded">--feedback</code>: <code className="bg-white px-1 rounded">ros2 action send_goal /navigate_to_position ... --feedback</code>
            </p>
          </div>
        </LectureSlide>

        {/* Slide 20: Cancel Server Side */}
        <LectureSlide id="slide-20" title="Cancel: Server Side" subtitle="Handling Cancellation Requests" icon={XCircle}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              To support cancellation, add a <code className="bg-zinc-100 px-2 py-1 rounded">cancel_callback</code> and check for cancel requests in the execute loop:
            </p>
          </div>

          <div className="space-y-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Step 1: Add cancel_callback</h4>
              <CodeBlock 
                filename="navigation_server.py"
                code={cancelServerCode}
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Step 2: Check for cancel in execute loop</h4>
              <CodeBlock 
                filename="navigation_server.py"
                code={cancelExecuteCode}
              />
            </div>
          </div>

          <div className="mt-6 grid md:grid-cols-2 gap-4">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h5 className="font-bold text-blue-900 mb-2">cancel_callback</h5>
              <p className="text-blue-800 text-sm">Decides whether to allow cancellation. Return <code className="bg-white px-1 rounded">CancelResponse.ACCEPT</code> or <code className="bg-white px-1 rounded">CancelResponse.REJECT</code>.</p>
            </div>
            <div className="p-4 bg-amber-50 border border-amber-200 rounded-lg">
              <h5 className="font-bold text-amber-900 mb-2">is_cancel_requested</h5>
              <p className="text-amber-800 text-sm">Check this inside your loop. If True, call <code className="bg-white px-1 rounded">goal_handle.canceled()</code>, return result, and stop.</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 21: Cancel Client Side */}
        <LectureSlide id="slide-21" title="Cancel: Client Side" subtitle="Requesting Cancellation" icon={RotateCcw}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              The client cancels a goal by calling <code className="bg-zinc-100 px-2 py-1 rounded">goal_handle.cancel_goal_async()</code>. Here&apos;s a client that cancels after 3 seconds:
            </p>
          </div>

          <CodeBlock 
            filename="navigation_client.py (with cancellation)"
            code={cancelClientCode}
          />

          <div className="mt-6">
            <h4 className="font-bold text-zinc-900 mb-3">Terminal output (cancelled after 3 seconds)</h4>
            <TerminalBlock 
              command="ros2 run my_robot_pkg navigation_client"
              output={`[INFO] [navigation_client]: Sending goal: navigate to (25.0, 15.0)
[INFO] [navigation_client]: Goal accepted!
[INFO] [navigation_client]: Position: (1.7, 1.0) | 27.2m left | 7%
[INFO] [navigation_client]: Position: (3.4, 2.1) | 25.1m left | 14%
[INFO] [navigation_client]: Position: (5.1, 3.1) | 22.9m left | 21%
[INFO] [navigation_client]: Requesting cancellation...
[INFO] [navigation_client]: CANCELLED at (5.1, 3.1)`}
              title="Terminal"
            />
          </div>

          <div className="mt-4 p-4 bg-zinc-50 rounded-lg">
            <p className="text-zinc-700 text-sm">
              <strong>Real-world use:</strong> Cancel navigation if a sensor detects an obstacle, if the user presses a stop button, or if a higher-priority task arrives.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 22: CLI Tools */}
        <LectureSlide id="slide-22" title="CLI Tools for Actions" subtitle="Inspect and Debug" icon={Wrench}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              ROS 2 provides <code className="bg-zinc-100 px-2 py-1 rounded">ros2 action</code> commands for working with actions:
            </p>
          </div>

          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">List all active actions</h4>
              <TerminalBlock 
                command="ros2 action list"
                output={`/navigate_to_position`}
                title="Terminal"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Show action type</h4>
              <TerminalBlock 
                command="ros2 action list -t"
                output={`/navigate_to_position [my_robot_interfaces/action/NavigateToPosition]`}
                title="Terminal"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Get action info</h4>
              <TerminalBlock 
                command="ros2 action info /navigate_to_position"
                output={`Action: /navigate_to_position
Action clients: 1
    /navigation_client
Action servers: 1
    /navigation_server`}
                title="Terminal"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Send a goal with feedback</h4>
              <TerminalBlock 
                command={`ros2 action send_goal /navigate_to_position my_robot_interfaces/action/NavigateToPosition "{target_x: 10.0, target_y: 6.0}" --feedback`}
                output={`Waiting for an action server to become available...
Sending goal: ...
Feedback:
  current_x: 2.0
  current_y: 1.2
  distance_remaining: 9.4
  completion_percentage: 20.0
Feedback:
  ...
Result: ...
Goal finished with status: SUCCEEDED`}
                title="Terminal"
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 23: Summary */}
        <LectureSlide id="slide-23" title="Summary" subtitle="What You Learned" icon={Layout}>
          <div className="grid md:grid-cols-2 gap-6 mb-8">
            <div className="space-y-4">
              <div className="p-4 bg-blue-50 rounded-lg">
                <h4 className="font-bold text-blue-900 mb-2">Action Interface</h4>
                <div className="font-mono text-xs text-blue-800">
                  Goal (top) --- Result (middle) --- Feedback (bottom)
                </div>
              </div>

              <div className="p-4 bg-green-50 rounded-lg">
                <h4 className="font-bold text-green-900 mb-2">Action Server</h4>
                <div className="font-mono text-xs text-green-800">
                  goal_callback → accept/reject<br />
                  execute_callback → do work + publish feedback<br />
                  cancel_callback → handle cancellation
                </div>
              </div>
            </div>

            <div className="space-y-4">
              <div className="p-4 bg-purple-50 rounded-lg">
                <h4 className="font-bold text-purple-900 mb-2">Action Client</h4>
                <div className="font-mono text-xs text-purple-800">
                  send_goal_async() → goal_response_callback<br />
                  → result_callback + feedback_callback<br />
                  cancel_goal_async() to cancel
                </div>
              </div>

              <div className="p-4 bg-amber-50 rounded-lg">
                <h4 className="font-bold text-amber-900 mb-2">When to Use Actions</h4>
                <div className="text-xs text-amber-800">
                  Long-running tasks (seconds to minutes) where you need progress updates and/or cancellation.
                </div>
              </div>
            </div>
          </div>

          <div className="p-6 bg-zinc-900 text-white rounded-xl">
            <h5 className="font-bold mb-4">All Three Communication Types</h5>
            <div className="grid md:grid-cols-3 gap-4 text-center">
              <div className="p-3 bg-zinc-800 rounded-lg">
                <div className="text-blue-400 font-bold mb-1">Topics</div>
                <div className="text-xs text-zinc-400">Streaming data</div>
                <div className="text-xs text-zinc-500 mt-1">Lectures 2-3</div>
              </div>
              <div className="p-3 bg-zinc-800 rounded-lg">
                <div className="text-green-400 font-bold mb-1">Services</div>
                <div className="text-xs text-zinc-400">Quick request/response</div>
                <div className="text-xs text-zinc-500 mt-1">Lectures 4-5</div>
              </div>
              <div className="p-3 bg-zinc-800 rounded-lg">
                <div className="text-purple-400 font-bold mb-1">Actions</div>
                <div className="text-xs text-zinc-400">Long tasks with feedback</div>
                <div className="text-xs text-zinc-500 mt-1">Lecture 7</div>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 24: Homework */}
        <LectureSlide id="slide-24" title="Homework" subtitle="Build a PickUpPackage Action" icon={Award}>
          <div className="mb-6 p-4 bg-zinc-900 text-white rounded-xl text-center">
            <p className="text-lg">
              Create an action for the warehouse robot&apos;s package pickup process.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-6 mb-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">1. Define the interface</h4>
              <CodeBlock 
                filename="PickUpPackage.action"
                code={homeworkInterfaceCode}
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">2. Server logic (hint)</h4>
              <CodeBlock 
                filename="pickup_server.py"
                code={homeworkServerHintCode}
              />
            </div>
          </div>

          <div className="space-y-4">
            <h4 className="font-bold text-zinc-900">Requirements:</h4>
            <div className="grid md:grid-cols-2 gap-4">
              <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
                <h5 className="font-bold text-blue-900 mb-2">Action Server</h5>
                <ul className="text-blue-800 text-sm space-y-1">
                  <li>• Reject empty package_id in goal_callback</li>
                  <li>• Simulate 5 phases (1 second each)</li>
                  <li>• Publish feedback with current_step and percentage</li>
                  <li>• Support cancellation (stop mid-pickup)</li>
                </ul>
              </div>
              <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
                <h5 className="font-bold text-green-900 mb-2">Action Client</h5>
                <ul className="text-green-800 text-sm space-y-1">
                  <li>• Send a goal with a package ID</li>
                  <li>• Display feedback as it arrives</li>
                  <li>• Print the final result</li>
                  <li>• Handle all three outcomes (success, cancel, abort)</li>
                </ul>
              </div>
            </div>
          </div>

          <div className="mt-6 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <h4 className="font-bold text-amber-900 mb-2">Bonus Challenge</h4>
            <p className="text-amber-800 text-sm">
              Add a <code className="bg-white px-1 rounded">gripper_speed</code> parameter (from Lecture 6!) to control the delay between pickup phases. Default 1.0 seconds, configurable at runtime.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 25: Next Lecture */}
        <LectureSlide id="slide-25" title="Next Lecture" subtitle="Launch Files - Composing Systems" icon={ArrowRight}>
          <div className="p-8 bg-gradient-to-br from-zinc-900 to-zinc-800 text-white rounded-2xl">
            <h3 className="text-2xl font-bold mb-6">Lecture 8: Launch Files</h3>
            
            <div className="grid md:grid-cols-2 gap-6 mb-8">
              <div>
                <h4 className="font-bold text-zinc-300 mb-3">The Problem</h4>
                <p className="text-zinc-400 text-sm">
                  We now have many nodes: task_manager, navigation_server, battery_monitor, sensor_filter... Starting each one manually in a separate terminal is painful. What about parameters for each node?
                </p>
              </div>
              <div>
                <h4 className="font-bold text-zinc-300 mb-3">The Solution: Launch Files</h4>
                <p className="text-zinc-400 text-sm">
                  Launch files let you start your entire robot system with a single command — all nodes, parameters, and configurations in one place.
                </p>
              </div>
            </div>

            <div className="space-y-4">
              <h4 className="font-bold text-zinc-300">What You&apos;ll Learn:</h4>
              <div className="grid md:grid-cols-3 gap-4">
                <div className="p-3 bg-zinc-800/50 rounded-lg">
                  <Rocket className="text-blue-400 mb-2" size={20} />
                  <p className="text-sm">Launch multiple nodes at once</p>
                </div>
                <div className="p-3 bg-zinc-800/50 rounded-lg">
                  <Sliders className="text-green-400 mb-2" size={20} />
                  <p className="text-sm">Pass parameters via launch</p>
                </div>
                <div className="p-3 bg-zinc-800/50 rounded-lg">
                  <Network className="text-purple-400 mb-2" size={20} />
                  <p className="text-sm">Compose the full system</p>
                </div>
              </div>
            </div>
          </div>

          <div className="mt-8 text-center text-zinc-500">
            <p>Complete your homework before the next session.</p>
            <p className="mt-2 font-medium text-zinc-700">See you in Lecture 8!</p>
          </div>
        </LectureSlide>

      </main>
    </div>
  );
}
