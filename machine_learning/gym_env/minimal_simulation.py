<<<<<<< HEAD
import pygame, math, itertools
=======
import pygame, math
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2

class Entity:
    def __init__(self, pos, identifier, discrete_pos):
        self.discrete_position = discrete_pos
        self.position = pos
        self.identifier = identifier


class Robot(Entity):
    def __init__(self, pos, identifier, discrete_pos):
        super().__init__(pos, identifier, discrete_pos)
        self.current_in = None
        self.carring = None
        self.traveled_distance = 0

        self.moved_to_same_place_callback = None
        self.already_have_puck_callback = None
        self.have_no_puck_to_pick_callback = None
        self.have_no_puck_to_leave_callback = None
        self.tried_leaving_puck_callback = None
        self.left_puck_callback = None
        self.about_to_pick_puck_callback = None

    def move_to(self, interest_point):
        if not isinstance(interest_point, InterestPoint):
            raise ValueError("!!")

        if self.current_in == interest_point:
            if self.moved_to_same_place_callback is not None:
                self.moved_to_same_place_callback(self)

        if self.current_in != None:
            a = self.current_in.discrete_position[0] - interest_point.discrete_position[0]
            b = self.current_in.discrete_position[1] - interest_point.discrete_position[1]
            self.traveled_distance = math.sqrt((a)**2 + (b)**2)

        self.position = [interest_point.position[0], interest_point.position[1]]
        self.discrete_position = interest_point.discrete_position

        if self.carring:
            self.carring.position = [self.position[0], self.position[1]]
        self.current_in = interest_point
        # print("- Robot moved to %s" %(self.current_in.identifier))

    def try_picking_puck(self):
        self.traveled_distance = 0

        s = "- Robot tried to pick puck at " + self.current_in.identifier

        if self.carring is not None:
            s += ", but it was already with a " + self.carring.identifier

            if self.already_have_puck_callback is not None:
                self.already_have_puck_callback(self)

            # print(s)

            return False

<<<<<<< HEAD

=======
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2
        if not (isinstance(self.current_in, Machine) or isinstance(self.current_in, DistributionCenter)):
            s += ", but it wasn't at a machine or a dc"

            # print(s)
            return False

        if self.about_to_pick_puck_callback:
            self.about_to_pick_puck_callback(self)

        self.carring = self.current_in.remove_puck()

        if self.carring is not None:
            s += ", sucessed, and got " + self.carring.identifier
            self.carring.position = [self.position[0], self.position[1]]

            # print(s)
            return True
        else:
            s += ", but it had no pucks"

            if self.have_no_puck_to_pick_callback is not None:
                self.have_no_puck_to_pick_callback(self)

            # print(s)

            return False

    def leave_puck(self):
        self.traveled_distance = 0

        if self.carring is None:
            # print("- Robot tried to leave the puck, but it didn't has any")

            if self.have_no_puck_to_leave_callback is not None:
                self.have_no_puck_to_leave_callback(self)

            return False

        # print("- Robot tried to leave the puck and sucessed")
        self.current_in.add_puck(self.carring)
        self.carring = None

        if self.left_puck_callback is not None:
            self.left_puck_callback(self)

        return True


class Puck(Entity):
    RED = 0
    YELLOW = 1
    BLUE = 2

    def __init__(self, pos, c, identifier, discrete_pos):
        super().__init__(pos, identifier, discrete_pos)
        self.color = c

class InterestPoint(Entity):
    def __init__(self, pos, identifier, discrete_pos):
        super().__init__(pos, identifier, discrete_pos)

        self.pucks = []
        self.is_empty_callback = None
        self.received_puck_callback = None
        self.create_puck_callback = None

    def rearrange_pucks(self):
        for p, i in zip(self.pucks, range(len(self.pucks))):
            p.position = [self.position[0] - 45 + i * 10 , self.position[1] + 50]

    def add_puck(self, puck):
        self.pucks.append(puck)

        if self.received_puck_callback:
            self.received_puck_callback(self)

        self.rearrange_pucks()

    def remove_puck(self):
        if self.pucks:
            p = self.pucks.pop(0)

            if self.is_empty_callback:
                self.is_empty_callback(self)

            if self.create_puck_callback:
                self.create_puck_callback(self)

            self.rearrange_pucks()

            return p
        else:
            return None

class Machine(InterestPoint):
    RED = 0
    YELLOW = 1
    BLUE = 2

    def __init__(self, pos, color, identifier, discrete_pos):
        super().__init__(pos, identifier, discrete_pos)

        self.color = color

class DistributionCenter(InterestPoint):
    def __init__(self, pos, identifier, discrete_pos):
        super().__init__(pos, identifier, discrete_pos)

class Dock(InterestPoint):
    def __init__(self, pos, identifier, discrete_pos):
        super().__init__(pos, identifier, discrete_pos)


class Map:
    def __init__(self, *entities):
        # Defined in meters
        self.__dimensions = (4, 4)

        self.entities = entities

        self.__graphics_manager = GraphicsManager()

        self.total_pucks = 3

    def render(self):
        self.__graphics_manager.render(self.entities)
<<<<<<< HEAD
        pass
=======
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2

class GraphicsManager:
    screen = None

    class Image:
        def __init__(self, path):
            self.__image = pygame.image.load(path)

        def draw(self, pos):
            r = self.__image.get_rect()

            GraphicsManager.screen.blit(self.__image,
                [pos[0] - r.w/2, pos[1] - r.h/2],
                r)

    def __init__(self):
        pygame.init()

        GraphicsManager.screen = pygame.display.set_mode((800, 800),
            pygame.HWSURFACE)

        GraphicsManager.screen.fill((255, 255, 255))

        self.font = pygame.font.SysFont("monospace", 18)
        self.images = {}
        self.images["dc"] = GraphicsManager.Image("images/dc.png")
        self.images["machine"] = GraphicsManager.Image("images/machine.png")
        self.images["robot"] = GraphicsManager.Image("images/robot.png")
        self.images["dock"] = GraphicsManager.Image("images/dock.png")

        self.images["red_puck"] = GraphicsManager.Image("images/red_puck.png")
        self.images["yellow_puck"] = GraphicsManager.Image("images/yellow_puck.png")
        self.images["blue_puck"] = GraphicsManager.Image("images/blue_puck.png")

    def render(self, entities):
        for e in entities:
            i = None

            if not isinstance(e, Entity):
                return

            if isinstance(e, DistributionCenter):
                i = "dc"
            elif isinstance(e, Machine):
                i = "machine"
            elif isinstance(e, Robot):
                i = "robot"
            elif isinstance(e, Dock):
                i = "dock"
            elif isinstance(e, Puck):
                i = "_puck"
                if e.color == Puck.RED:
                    i = "red" + i
                if e.color == Puck.YELLOW:
                    i = "yellow" + i
                if e.color == Puck.BLUE:
                    i = "blue" + i

            # Draw image
            self.images[i].draw(e.position)

            # Draw texts
            if isinstance(e, InterestPoint):
                tl = self.font.render(e.identifier + " " + str(e.discrete_position), True, (0, 0, 0))
                p = (e.position[0] - tl.get_rect().w/2, e.position[1] - 58)

            elif isinstance(e, Puck):
                tl = self.font.render(e.identifier.split()[2], True, (255, 255, 255))
                p = (e.position[0] - tl.get_rect().w/2, e.position[1] - tl.get_rect().h/2)

            GraphicsManager.screen.blit(tl, p)

        pygame.display.flip()
        GraphicsManager.screen.fill((255, 255, 255))

MAP = Map()
class Env1:
    total_traveled_distance = 0
    t_reward = 0
    total_optimal_matches = 0
    def __init__(self, **params):
        Env1.total_traveled_distance = 0

        self.dock = Dock((65, 735), "dock", params["dock_position"])

        self.red_machine = Machine((735, 735),
            Machine.RED,
            "red machine",
            params["red_machine_pos"])

        self.blue_machine = Machine((610, 735),
            Machine.BLUE,
            "blue machine",
            params["blue_machine_pos"])

        self.yellow_machine = Machine((485, 735),
            Machine.YELLOW,
            "yellow machine",
            params["yellow_machine_pos"])

        self.steps = 0

        p1 = Puck((0, 0), Puck.BLUE, "blue puck 0", (0,0))
        self.blue_machine.add_puck(p1)
        p2 = Puck((0, 0), Puck.RED, "red puck 1", (0,0))
        self.red_machine.add_puck(p2)
        p3 = Puck((0, 0), Puck.YELLOW, "yellow puck 2", (0,0))
        self.yellow_machine.add_puck(p3)

        self.machines = [self.red_machine,
                         self.blue_machine,
                         self.yellow_machine]

        self.dcs = [DistributionCenter((735, 65), "dc 1", params["dc1_pos"]),
                    DistributionCenter((610, 65), "dc 2", params["dc2_pos"]),
                    DistributionCenter((485, 65), "dc 3", params["dc3_pos"]),
                    DistributionCenter((360, 65), "dc 4", params["dc4_pos"]),
                    DistributionCenter((235, 65), "dc 5", params["dc5_pos"]),
                    DistributionCenter((110, 65), "dc 6", params["dc6_pos"])]

        self.interest_points = [self.dock] + self.machines + self.dcs

        self.robot = Robot((65, 735), "robot", (0, 0))
        self.robot.move_to(self.dock)

        MAP.entities = [self.dock,
                          self.red_machine,
                          self.blue_machine,
                          self.yellow_machine,
                          self.robot] + self.dcs + [p1, p2, p3]

        self.ever_received_puck = [0, 0, 0, 0, 0, 0]
        self.color_patterns = [-1, -1, -1]

        def create_puck_callback(machine):
            if machine.color == Machine.RED:
                p = Puck((0, 0), Puck.RED, "red puck %d" %MAP.total_pucks, (0, 0))

            elif machine.color == Machine.BLUE:
                p = Puck((0, 0), Puck.BLUE, "blue puck %d" %MAP.total_pucks, (0, 0))

            elif machine.color == Machine.YELLOW:
                p = Puck((0, 0), Puck.YELLOW, "yellow puck %d" %MAP.total_pucks, (0, 0))

            MAP.entities.append(p)
            machine.add_puck(p)
            MAP.total_pucks += 1

        def received_puck_callback(dc):
            pass


        for dc in self.dcs:
            dc.received_puck_callback = received_puck_callback

        for m in self.machines:
            m.create_puck_callback = create_puck_callback


        def moved_to_same_place_callback(r):
            Env1.t_reward = -2

        def already_have_puck_callback(r):
            Env1.t_reward = -1

        def have_no_puck_to_pick_callback(r):
            Env1.t_reward = -1

        def have_no_puck_to_leave_callback(r):
            Env1.t_reward = -2

        def left_puck_callback(r):
            dc = r.current_in
<<<<<<< HEAD
            # Env1.t1t_reward = 0

            if isinstance(r.current_in, Machine) or isinstance(r.current_in, Dock):
                # Reward for leaving the puck on the wrong place (machine or dock)
                Env1.t_reward = 0
=======
            Env1.t1t_reward = 0

            if isinstance(r.current_in, Machine) or isinstance(r.current_in, Dock):
                # Reward for leaving the puck on the wrong place (machine or dock)
                Env1.t1t_reward = -2
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2

            elif len(r.current_in.pucks) == 1 and self.ever_received_puck[self.dcs.index(dc)] == 0:
                self.ever_received_puck[self.dcs.index(dc)] = 1

                dc_id = self.dcs.index(dc)

                if self.color_patterns[int(dc_id/2)] == -1:
                    cp = self.color_patterns[int(dc_id/2) + 1:] + self.color_patterns[:int(dc_id/2)]

                    if cp[0] != dc.pucks[0].color and cp[1] != dc.pucks[0].color:
                        # The robot defined a pattern for the dc
                        self.color_patterns[int(dc_id/2)] = dc.pucks[0].color
                        Env1.t_reward = 10

                    else:
                        # This color was already setted for another pattern
                        Env1.t_reward = 0

                elif self.color_patterns[int(dc_id/2)] == dc.pucks[0].color:
                    # The color matched with the pattern
                    Env1.t_reward = 10

                else:
                    # Leave a puck which did not match with the pattern
                    Env1.t_reward = 0

            else:
                Env1.t_reward = 0


        def about_to_pick_puck_callback(r):
            if isinstance(r.current_in, DistributionCenter):
                Env1.t_reward = -4

        self.robot.moved_to_same_place_callback = moved_to_same_place_callback
        self.robot.already_have_puck_callback = already_have_puck_callback
        self.robot.have_no_puck_to_pick_callback = have_no_puck_to_pick_callback
        self.robot.have_no_puck_to_leave_callback = have_no_puck_to_leave_callback
        self.robot.left_puck_callback = left_puck_callback
        self.robot.about_to_pick_puck_callback = about_to_pick_puck_callback

<<<<<<< HEAD
        _, _, self.initial_state, _ = self.observe()
=======
        _, _, self.initial_state = self.observe()
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2

    def render(self):
        MAP.render()

    def observe(self):
        state =  [0]                      # Holding puck (0, 1 red, 2 blue, 3 yellow)
        state += self.robot.discrete_position  # Robot at (1 - 3 machine, 4 - 10 dcs)
        state+= [0,    # cd 1 (0, 1 red, 2 blue, 3 yellow)
                 0,    # cd 2 (0, 1 red, 2 blue, 3 yellow)
                 0,    # cd 3 (0, 1 red, 2 blue, 3 yellow)
                 0,    # cd 4 (0, 1 red, 2 blue, 3 yellow)
                 0,    # cd 5 (0, 1 red, 2 blue, 3 yellow)
                 0]    # cd 6 (0, 1 red, 2 blue, 3 yellow)
        state += self.dcs[0].discrete_position # cd 0 xy
        state += self.dcs[1].discrete_position # cd 0 xy
        state += self.dcs[2].discrete_position # cd 0 xy
        state += self.dcs[3].discrete_position # cd 0 xy
        state += self.dcs[4].discrete_position # cd 0 xy
        state += self.dcs[5].discrete_position # cd 0 xy
        state += [1,    # Machine 1 color
                  2,
                  3]
        state += self.red_machine.discrete_position
        state += self.blue_machine.discrete_position
        state += self.yellow_machine.discrete_position

        c = self.robot.carring
        if c is not None:
            c = c.color
            if c == Puck.RED:
                state[0] = 1
            elif c == Puck.BLUE:
                state[0] = 2
            elif c == Puck.YELLOW:
                state[0] = 3
        else:
            state[0] = 0

        final = 0
        for dc, i in zip(self.dcs, range(0, len(self.dcs))):
            i = i + 3
            if dc.pucks:
                final += 1
                c = dc.pucks[0].color
                if c == Puck.RED:
                    state[i] = 1
                elif c == Puck.BLUE:
                    state[i] = 2
                elif c == Puck.YELLOW:
                    state[i] = 3
            else:
                state[i] = 0

        terminal = False
<<<<<<< HEAD

        reward = Env1.t_reward - self.robot.traveled_distance/1000
=======
        reward = Env1.t_reward - self.robot.traveled_distance/100
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2
        Env1.total_traveled_distance += self.robot.traveled_distance
        if self.steps == 50:
            terminal = True

<<<<<<< HEAD
        all_finished = False
=======
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2
        if final == 6 or self.steps == 60:
            terminal = True

            if self.dcs[0].pucks[0].color == self.dcs[1].pucks[0].color:
                c1 = self.dcs[0].pucks[0].color
                if (self.dcs[2].pucks[0].color == self.dcs[3].pucks[0].color) and self.dcs[2].pucks[0].color != c1:
                    c2 = self.dcs[2].pucks[0].color
                    if (self.dcs[4].pucks[0].color == self.dcs[5].pucks[0].color) and self.dcs[5].pucks[0].color != c1 and self.dcs[5].pucks[0].color != c2:
                        print("Full match in %s steps" %(self.steps))

                        if self.steps == 24:
                            Env1.total_optimal_matches += 1
                            print("Optimal match! Counting: %s" %Env1.total_optimal_matches)
<<<<<<< HEAD
                            if Env1.total_optimal_matches == 20:
                                all_finished = True
=======
                            # if Env1.total_optimal_matches == 5:
                            #     exit()
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2
                        else:
                            Env1.total_optimal_matches = 0

        Env1.t_reward = 0

<<<<<<< HEAD
        return (terminal, reward, state, all_finished)
=======
        return (terminal, reward, state)
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2


    def act(self, action_id):
        if   action_id < len(self.interest_points):
            self.robot.move_to(self.interest_points[action_id])
        elif action_id == len(self.interest_points):
            self.robot.try_picking_puck()
        elif action_id == len(self.interest_points) + 1:
            self.robot.leave_puck()

        self.steps += 1

# if  __name__ == "__main__":
#     e = Env1(red_machine_pos=    [18, 18],
#              blue_machine_pos=   [18, 17],
#              yellow_machine_pos= [18, 16],
#              dock_position=      [18, 0],
#              dc1_pos=            [0, 18],
#              dc2_pos=            [0, 17],
#              dc3_pos=            [0, 16],
#              dc4_pos=            [0, 15],
#              dc5_pos=            [0, 14],
#              dc6_pos=            [0, 13])
#
#     while True:
#         e.render()
#         a = int(input())
#         e.act(a)
#         t, r, s = e.observe()
#
#         print("Traveled: %s\nState: %s\nReward: %s" %(e.robot.traveled_distance, s, r))


if __name__ == "__main__":
    from tensorforce.agents import PPOAgent
    import tensorflow as tf

    gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.2)

    agent = PPOAgent(
        states=dict(type='float', shape=(30)), #shape=[4, 9, 4, 4, 4, 4, 4, 4]),
        actions=dict(type='int', num_actions=12),
        network=[
            dict(type='dense', size=64, activation='tanh'),
            dict(type='dense', size=64, activation='tanh'),
<<<<<<< HEAD
            dict(type='dense', size=64, activation='tanh'),
=======
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2
            dict(type='dense', size=64, activation='tanh')
        ],
        update_mode=dict(
            batch_size=50
        ),
        execution=dict(
            type='single',
            session_config=tf.ConfigProto(gpu_options=gpu_options),
            distributed_spec=None
        ),
        step_optimizer=dict(type='adam',learning_rate=1e-4),
        baseline_mode="states",
        baseline=dict(
            type="mlp",
<<<<<<< HEAD
            sizes=[128, 128]
=======
            sizes=[128]
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2
        ),
        baseline_optimizer=dict(
            type="multi_step",
            optimizer=dict(
                type="adam",
                learning_rate=1e-4
            ),
            num_steps=10
        )
    )


    try:
        agent.restore_model(directory="data", file="data")
        print("Data loaded")
    except:
        print("Couldn't load data")

<<<<<<< HEAD
    import time, numpy, csv
    from random import randint
=======
    import random, time, numpy, csv
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2

    rt = [float(row[0]) for row in csv.reader(open("reward_history.csv"), delimiter=",")]

    episodes_r = []
    episodes_d = []
<<<<<<< HEAD
    for e in range(1000000):
        # print("New configuration")
        # point_cloud = list(itertools.permutations(range(0, 19), 2))
        #
        # points = [point_cloud.pop(randint(0, len(point_cloud) - 1)) for _ in range(10)]
        #
        # env = Env1(red_machine_pos=    points.pop(randint(0, len(points) - 1)),
        #            blue_machine_pos=   points.pop(randint(0, len(points) - 1)),
        #            yellow_machine_pos= points.pop(randint(0, len(points) - 1)),
        #            dock_position=      points.pop(randint(0, len(points) - 1)),
        #            dc1_pos=            points.pop(randint(0, len(points) - 1)),
        #            dc2_pos=            points.pop(randint(0, len(points) - 1)),
        #            dc3_pos=            points.pop(randint(0, len(points) - 1)),
        #            dc4_pos=            points.pop(randint(0, len(points) - 1)),
        #            dc5_pos=            points.pop(randint(0, len(points) - 1)),
        #            dc6_pos=            points.pop(randint(0, len(points) - 1)))

=======
    for e in range(100000):
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2
        env = Env1(red_machine_pos=    [18, 18],
                   blue_machine_pos=   [18, 17],
                   yellow_machine_pos= [18, 16],
                   dock_position=      [18, 0],
                   dc1_pos=            [0, 18],
                   dc2_pos=            [0, 17],
                   dc3_pos=            [0, 16],
                   dc4_pos=            [0, 15],
                   dc5_pos=            [0, 14],
                   dc6_pos=            [0, 13])

        total_r = 0
        total_d = 0
        terminal = False
        s = env.initial_state
        while not terminal:
            env.render()

<<<<<<< HEAD
            env.act(agent.act(s, deterministic=True))
            terminal, r, s, all_finished = env.observe()
            total_r +=r
            total_d += env.total_traveled_distance
            agent.observe(reward=r, terminal=terminal)
            input("!")
=======
            env.act(agent.act(s)) #, deterministic=True))
            terminal, r, s = env.observe()
            total_r +=r
            total_d += env.total_traveled_distance
            agent.observe(reward=r, terminal=terminal)
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2

        episodes_r.append(total_r)
        episodes_d.append(env.total_traveled_distance)

        # print("Total episode reward: %s\nTotal distance:%s\n" %(total_r, env.total_traveled_distance))
        # rt.append(total_r)
        # print("Total avg: %s" %(sum(rt)/len(rt)))

        if e%10 == 0:
            print("Avg reward (10): %s\nAvg distance: %s" %(sum(episodes_r[-10:])/10, sum(episodes_d[-10:])/10))

        if e%100 == 0:
            agent.save_model(directory="data/data", append_timestep=False)
            print("Saved!")

<<<<<<< HEAD
        if all_finished:
            agent.save_model(directory="data/data", append_timestep=False)
            print("All finished")
            exit()

=======
>>>>>>> bc22da9c4c7f15ed83c05bf9792dc39395fe87f2
        with open("reward_history.csv", "a") as csvfile:
            writer = csv.writer(csvfile)

            writer.writerow([total_r, total_d])
