"""Mangaes the main make_coffee action and routine.

Coffee routine is as follows:
1. Waits to detect an empty cup placed in the detection area (coffee_start subscriber)
2. Pick up Filter from filter stand (pick_filter action)
3. Place Filter on Pot (place_filter_in_pot action)
4. Pick up scoop and dump coffee grounds in filter (scoop action)
5. Wait for boiling
6. Pick up Kettle (pick_kettle action)
7. Pour water from kettle (pour_action action)
8. Place Kettle on kettle stand (place_kettle action)
9. Wait for coffee grounds to soak
10. Pick up Filter from Pot (pick_filter_in_pot action)
11. Place Filter on filter stand (place_filter action)
12. Pick up Pot from pot stand (pick_pot action)
13. Pour Coffee (pour_action action)
14. Put Pot on pot stand (place_pot action)

Subscriptions:
  coffee_start (Empty) - indicates that a cup has been dectected and the robot should start making coffee

Service Clients:
  delay (DelayTime) - allows for delays between steps

Action Clients:
  pick_kettle (EmptyAction) - picks up the kettle
  place_kettle (EmptyAction) - places the kettle
  pick_pot (EmptyAction) - picks up the pot
  place_pot (EmptyAction) - places the pot 
  pour_kettle (EmptyAction) - pours the kettle
  scoop (EmptyAction) - picks up the scoop, move it to the pot, dumps the grounds, the puts the scoop back
  pick_filter (EmptyAction) - picks up the filter from its holder
  place_filter (EmptyAction) - places the filter on its holder
  place_filter_in_pot (EmptyAction) - places the filter in the pot
  pick_filter_in_pot (EmptyAction) - picks up the filter from the pot
  pour_pot (EmptyAction) - pours the coffee from the pot to the cup

Action Servers:
  make_coffee (EmptyAction) - The routine to make a cup of coffee
"""


import rclpy
from rclpy.node import Node
from moveit_wrapper.moveitapi import MoveItApi
from std_msgs.msg import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
from botrista_interfaces.action import EmptyAction
from botrista_interfaces.srv import DelayTime
from rclpy.action import ActionClient


class Botrista(Node):
    def __init__(self):
        """Initializes the botrista node.
        """
        super().__init__("botrista")

        # start action client for pick_kettle action
        self.action_client_pick_kettle = ActionClient(
            self, EmptyAction, "pick_kettle", callback_group=ReentrantCallbackGroup()
        )

        # start action client for place_kettle action
        self.action_client_place_kettle = ActionClient(
            self, EmptyAction, "place_kettle", callback_group=ReentrantCallbackGroup()
        )

        # start action client for pick_pot action
        self.action_client_pick_pot = ActionClient(
            self, EmptyAction, "pick_pot", callback_group=ReentrantCallbackGroup()
        )

        # start action client for place_pot action
        self.action_client_place_pot = ActionClient(
            self, EmptyAction, "place_pot", callback_group=ReentrantCallbackGroup()
        )

        # start action client for pour_action action
        self.action_client_pour_kettle = ActionClient(
            self, EmptyAction, "pour_kettle", callback_group=ReentrantCallbackGroup()
        )

        # start action client for scoop action
        self.action_client_scoop = ActionClient(
            self, EmptyAction, "scoop", callback_group=ReentrantCallbackGroup()
        )

        # start action client for pick_filter action
        self.action_client_pick_filter = ActionClient(
            self, EmptyAction, "pick_filter", callback_group=ReentrantCallbackGroup()
        )

        # start action client for place_filter action
        self.action_client_place_filter = ActionClient(
            self, EmptyAction, "place_filter", callback_group=ReentrantCallbackGroup()
        )

        # start action client for place_filter_in_pot action
        self.action_client_place_filter_in_pot = ActionClient(
            self,
            EmptyAction,
            "place_filter_in_pot",
            callback_group=ReentrantCallbackGroup(),
        )

        # start action client for pick_filter_in_pot action
        self.action_client_pick_filter_in_pot = ActionClient(
            self,
            EmptyAction,
            "pick_filter_in_pot",
            callback_group=ReentrantCallbackGroup(),
        )

        # start action client for pour_pot action
        self.pour_pot_client = ActionClient(
            self, EmptyAction, "pour_pot", callback_group=ReentrantCallbackGroup()
        )

        # Delay service client
        self.delay_client = self.create_client(
            DelayTime, "delay", callback_group=ReentrantCallbackGroup()
        )

        self.start_coffee_subscriber = self.create_subscription(
            Empty,
            "coffee_start",
            self.start_coffee_callback,
            10,
            callback_group=ReentrantCallbackGroup(),
        )

        # move it api to home robot
        self.moveit_api = MoveItApi(
            self,
            "panda_link0",
            "panda_hand_tcp",
            "panda_manipulator",
            "joint_states",
            "panda",
        )

        self.has_started = False

    async def start_coffee_callback(self, msg):
        """Callback function for the coffee_start subscriber. Triggers the make_coffee routine.
        """
        if not self.has_started:
            self.get_logger().warn("Starting coffee routine")
            self.has_started = True
            await self.make_coffee()

    async def make_coffee(self):
        """Calls the actions required to make a cup of coffee in order.
        """
        # 1. Turns on Kettle (action)
        # 2. Pick up Filter from filter stand (pick_filter action)
        goal2 = EmptyAction.Goal()
        result = await self.action_client_pick_filter.send_goal_async(goal2)
        await result.get_result_async()

        # 3. Place Filter on Pot (place_filter_in_pot action)
        goal3 = EmptyAction.Goal()
        result = await self.action_client_place_filter_in_pot.send_goal_async(goal3)
        await result.get_result_async()

        # 4. Scoop grounds (scoop action)
        goal4 = EmptyAction.Goal()
        result = await self.action_client_scoop.send_goal_async(goal4)
        await result.get_result_async()

        # # 5. Wait for boiling
        # 6. Pick up Kettle (pick_kettle action)
        goal6 = EmptyAction.Goal()
        result = await self.action_client_pick_kettle.send_goal_async(goal6)
        await result.get_result_async()

        # # 7. Pour water from kettle (pour_action action)
        goal7 = EmptyAction.Goal()
        result = await self.action_client_pour_kettle.send_goal_async(goal7)
        await result.get_result_async()

        # 8. Place Kettle on kettle stand (place_kettle action)
        goal8 = EmptyAction.Goal()
        result = await self.action_client_place_kettle.send_goal_async(goal8)
        await result.get_result_async()
        await self.moveit_api.go_home()
        await self.delay_client.call_async(DelayTime.Request(time=10.0))

        # 9. Wait for coffee grounds to soak
        # 10. Pick up Filter from Pot (pick_filter_in_pot action)
        goal10 = EmptyAction.Goal()
        result = await self.action_client_pick_filter_in_pot.send_goal_async(goal10)
        await result.get_result_async()

        # 11. Place Filter on filter stand (place_filter action)
        goal11 = EmptyAction.Goal()
        result = await self.action_client_place_filter.send_goal_async(goal11)
        await result.get_result_async()

        await self.moveit_api.go_home()

        # 12. Pick up Pot from pot stand (pick_pot action)
        goal12 = EmptyAction.Goal()
        result = await self.action_client_pick_pot.send_goal_async(goal12)
        await result.get_result_async()

        # 13. Pour Coffee (pour_action action)
        goal13 = EmptyAction.Goal()
        result = await self.pour_pot_client.send_goal_async(goal13)
        await result.get_result_async()

        await self.moveit_api.go_home()

        # 14. Put Pot on pot stand (place_pot action)
        goal14 = EmptyAction.Goal()
        result = await self.action_client_place_pot.send_goal_async(goal14)
        await result.get_result_async()

        await self.moveit_api.go_home()


def botrista_entry(args=None):
    rclpy.init(args=args)
    botrista = Botrista()
    rclpy.spin(botrista)
    rclpy.shutdown()
