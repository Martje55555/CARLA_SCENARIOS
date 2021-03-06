from __future__ import print_function

import argparse
from audioop import cross
from bdb import GENERATOR_AND_COROUTINE_FLAGS
import collections
import datetime
from distutils.command.build_scripts import first_line_re
from distutils.spawn import spawn
import glob
import logging
import math
from operator import is_
import os
import numpy.random as random
import re
import sys
import weakref
import time

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

walkers_list = []
all_id = []
cross_walks = []
walkers = []
vehicles = []

def find_weather_presets():
    """Method to find weather presets"""
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    def name(x): return ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

# Spawning Pedestrians givent the world, client and amount of pedestrians
def spawn_pedestrians(world, client, number_of_pedestrians):
    # add pedestrians to the world
    blueprintsWalkers = world.get_blueprint_library().filter("walker.pedestrian.*")
    walker_bp = random.choice(blueprintsWalkers)

    spawn_points = []
    for i in range(number_of_pedestrians):
        spawn_point = carla.Transform()
        spawn_point.location = cross_walks[0] #world.get_random_location_from_navigation()
        spawn_point.location.y = spawn_point.location.y + 6
        spawn_point.location.x = spawn_point.location.x + 2
        if (spawn_point.location != None):
            spawn_points.append(spawn_point)

    batch = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

    # apply the batch
    results = client.apply_batch_sync(batch, True)

    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})

    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))

    # apply the batch
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id
            walkers.append(results[i].actor_id)

    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
        #walkers.append(walkers_list[i]["id"])
    all_actors = world.get_actors(all_id)

    world.wait_for_tick()

    #if magnitude(npc.get_location() - av.get_location()) < threshold:
    for i in range(0, len(all_actors), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # random max speed
        all_actors[i].set_max_speed(1 + random.random())

class World(object):
    """ Class representing the surrounding environment """

    def __init__(self, carla_world, args, client):
        """Constructor method"""
        self._args = args
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.player = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self.client = client
        self.restart(args)

    def restart(self, args):
        """Restart the world"""
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_id = self.camera_manager.transform_index if self.camera_manager is not None else 0

        # get tesla vehicle from blueprint
        blueprint_library = self.world.get_blueprint_library()
        bp = blueprint_library.filter('model3')[0]

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0

            # spawn_point = self.map.get_waypoint(carla.Location(x=0, y=30, z=10))
            spawn_point = self.map.get_waypoint(carla.Location(x=-86.347275, y=24.404694, z=1.0))
            spawn_point.location.x = -86.347275
            spawn_point.location.y = 24.404694
            spawn_point.location.z = 1.0

            self.destroy()
            self.player = self.world.try_spawn_actor(bp, spawn_point.transform)
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()

            spawn_point = spawn_points[0]
            spawn_point.location.x = -86.347275
            spawn_point.location.y = 24.404694
            spawn_point.location.z = 1.0
            
            print(spawn_point.location)
            self.player = self.world.try_spawn_actor(bp, spawn_point)
            
            self.modify_vehicle_physics(self.player)

        # Set up the sensors.
        self.camera_manager = CameraManager(self.player)
        self.camera_manager.transform_index = cam_pos_id
        self.camera_manager.set_sensor(cam_index, notify=False)

        if self._args.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def spawn_vehicles_straight(self, radius, numbers_of_vehicles, x, y):
        # spawn_points = self.map.get_spawn_points()
        # np.random.shuffle(spawn_points)  # shuffle  all the spawn points
        # ego_location = self.player.get_location()
        # accessible_points = []
        # for spawn_point in spawn_points:
        #     dis = math.sqrt((ego_location.x-spawn_point.location.x)**2 + (ego_location.y-spawn_point.location.y)**2)
        #     # it also can include z-coordinate,but it is unnecessary
        #     if dis < radius:
        #         print(dis)
        #         accessible_points.append(spawn_point)

        vehicle_bps = self.world.get_blueprint_library().filter('vehicle.*.*')   # don't specify the type of vehicle
        vehicle_bps = [x for x in vehicle_bps if int(x.get_attribute('number_of_wheels')) == 4]  # only choose car with 4

        vehicle_list = []  # keep the spawned vehicle in vehicle_list, because we need to link them with traffic_manager
        # if len(accessible_points) < numbers_of_vehicles:
        #     # if your radius is relatively small,the satisfied points may be insufficient
        #     numbers_of_vehicles = len(accessible_points)

        accessible_points = self.map.get_spawn_points()

        for i in range(1):
            point = accessible_points[i]
            point.location.x = x
            point.location.y = y
            point.location.z = 1.0
            vehicle_bp = np.random.choice(vehicle_bps)
            try:
                print(point)
                vehicle = self.world.try_spawn_actor(vehicle_bp, point)
                vehicle_list.append(vehicle)
                vehicles.append(vehicle)
            except:
                print('failed')  # if failed, print the hints.
            
        # add those vehicles into trafficemanager, and set them to autopilot.
        tm = self.client.get_trafficmanager()  # create a TM object
        tm.global_percentage_speed_difference(10.0)  # set the global speed limitation
        tm_port = tm.get_port()  # get the port of tm. we need add vehicle to tm by this port
        tm.set_random_device_seed(9)
        tm.set_respawn_dormant_vehicles(True)
        tm.set_boundaries_respawn_dormant_vehicles(20, 500)
        for v in vehicle_list:  # set every vehicle's mode to be "normal"
            v.set_autopilot(True, tm_port)
            tm.ignore_lights_percentage(v, 0)
            tm.distance_to_leading_vehicle(v, 5.0)
            tm.vehicle_percentage_speed_difference(v, -15)
            tm.update_vehicle_lights(v, True)
            tm.set_route(v, ["Straight"])
        print(len(vehicle_list))

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def render(self, display):
        """Render world"""
        self.camera_manager.render(display)

    def destroy_sensors(self):
        """Destroy sensors"""
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self, world):
        """Destroys all actors"""
        actors = [
            self.camera_manager.sensor,
            self.player]
        # destroy player and sensors
        for actor in actors:
            if actor is not None:
                actor.destroy()
        # destroy walkers
        for actor in world.get_actors():
            if actor is not None and actor.type_id == "controller.ai.walker":
                actor.destroy()
        # destroy vehicles
        for vehicle in vehicles:
            if actor is not None:
                actor.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================

class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world):
        if isinstance(world.player, carla.Vehicle):
            pass
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0

    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True

    @staticmethod
    def _is_quit_shortcut(key):
        """Shortcut for quitting"""
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    """ Class for camera management"""

    def __init__(self, parent_actor):
        """Constructor method"""
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.spawn_point = carla.Transform(carla.Location(x=.20, y=0, z=1.10))

        self.transform_index = 1

        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}]]

        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            blp = bp_library.find(item[0])
            blp.set_attribute('fov', '110')
            if item[0].startswith('sensor.camera'):
                blp.set_attribute('image_size_x', str(600))
                blp.set_attribute('image_size_y', str(400))
            item.append(blp)
        self.index = None

    def toggle_camera(self):
        """Activate a camera"""
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        """Set a sensor"""
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else (
            force_respawn or (self.sensors[index][0] != self.sensors[self.index][0]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self.spawn_point,
                attach_to=self._parent)

            # We need to pass the lambda a weak reference to
            # self to avoid circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        self.index = index

    def render(self, display):
        """Render method"""
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

# ==============================================================================
# -- Game Loop ---------------------------------------------------------
# ==============================================================================

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    try:
        if args.seed:
            random.seed(args.seed)

        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)

        traffic_manager = client.get_trafficmanager()
        sim_world = client.get_world()

        if args.sync:
            settings = sim_world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            traffic_manager.set_synchronous_mode(True)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        world = World(client.get_world(), args, client)
        controller = KeyboardControl(world)

        # waypoints = world.map.generate_waypoints(0.1)
        # for w in waypoints:
        #     world.world.debug.draw_string(w.transform.location, 'O', draw_shadow=False,
        #                                color=carla.Color(r=255, g=0, b=0), life_time=120.0,
        #                                persistent_lines=True)

        if args.agent == "Basic":                                                                                                                                                       
            agent = BasicAgent(world.player)
        else:
            agent = BehaviorAgent(world.player, behavior=args.behavior)
                                                                                                                                                                                                                                                                                                                                                                                                                
        # Set the agent destination
        spawn_points = world.map.get_spawn_points()
        destination = random.choice(spawn_points).location
        print(destination)
        destination.x = 212
        destination.y = 326
        print(destination)

        end_destination = random.choice(spawn_points).location
        end_destination.x = 106.416290
        end_destination.y = -12.711931
        print(end_destination)

        agent.set_destination(start_location=destination, end_location=end_destination)
        first_crosswalk = world.player.get_location()
        first_crosswalk.x = 84
        first_crosswalk.y = 25
        first_crosswalk.z = 0.004

        cross_walks.append(first_crosswalk)
        
        clock = pygame.time.Clock()

        is_spawned = False

        clock = pygame.time.Clock()

        oldTime = time.time()

        index = 0
        # Getting different end destinations ##########################
        different_end_destinations = []

        #0
        end_destination = random.choice(spawn_points).location
        end_destination.x = 110.800049
        end_destination.y = 72.599747
        different_end_destinations.append(end_destination)

        # 1
        end_destination = random.choice(spawn_points).location
        end_destination.x = 90.432556
        end_destination.y = 12.643750
        different_end_destinations.append(end_destination)

        # 2
        end_destination = random.choice(spawn_points).location
        end_destination.x = 52.143875
        end_destination.y = 106.947296
        different_end_destinations.append(end_destination)

        # 3
        end_destination = random.choice(spawn_points).location
        end_destination.x = 113.648468
        end_destination.y = 4.688451
        different_end_destinations.append(end_destination)

        # 4
        end_destination = random.choice(spawn_points).location
        end_destination.x = 68.927277
        end_destination.y = 27.830568
        different_end_destinations.append(end_destination)

        #5
        end_destination = random.choice(spawn_points).location
        end_destination.x = 82.522911
        end_destination.y = 70.302856
        different_end_destinations.append(end_destination)

        #6
        end_destination = random.choice(spawn_points).location
        end_destination.x = 85.100761
        end_destination.y = 16.689871
        different_end_destinations.append(end_destination)

        # 7
        end_destination = random.choice(spawn_points).location
        end_destination.x = 87.605782
        end_destination.y = 130.068909
        different_end_destinations.append(end_destination)

        # 8
        end_destination = random.choice(spawn_points).location
        end_destination.x = 212
        end_destination.y = 326
        different_end_destinations.append(end_destination)

        # 9
        end_destination = random.choice(spawn_points).location
        end_destination.x = 110.632843
        end_destination.y = -4.862453
        different_end_destinations.append(end_destination)

        # 10
        end_destination = random.choice(spawn_points).location
        end_destination.x = -44.846107
        end_destination.y = 45.654007
        different_end_destinations.append(end_destination)

        # 11
        end_destination = random.choice(spawn_points).location
        end_destination.x = -69.321930
        end_destination.y = -58.023651
        different_end_destinations.append(end_destination)

        spawn_vehicle_at_spawn = False

        ####################################################

        while True:
            clock.tick()
            if args.sync:
                world.world.tick()
            else:
                world.world.wait_for_tick()
            if controller.parse_events():
                return

            world.render(display)
            pygame.display.flip()

            velocity = world.player.get_velocity()
            speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) # in m/s

            loc = world.player.get_location()

            # SPAWN PEDESTRIAN AT CROSSWALK and VEHICLE
            if loc.x < 55  and loc.x > 50 and loc.y < 25 and loc.y > 22 and is_spawned == False:
                spawn_pedestrians(world=world.world, client=client, number_of_pedestrians=1) 
                world.spawn_vehicles_straight(radius=50.0, numbers_of_vehicles=1, x=106, y=52)
                world.spawn_vehicles_straight(radius=50.0, numbers_of_vehicles=1, x=99,y=-22)
                is_spawned = True
            
            # SPAWN VEHICLES LEFT AND RIGHT OF SPAWNED PLAYER
            if spawn_vehicle_at_spawn == False:
                spawn_vehicle_at_spawn = True
                world.spawn_vehicles_straight(radius=50.0, numbers_of_vehicles=1, x=-52.498489, y=-11.581840)
                world.spawn_vehicles_straight(radius=50, numbers_of_vehicles=1, x=-45, y=63)
                spawn_vehicle_at_spawn = True

            # # SPAWN VEHICLE AT NEXT INTERSECTION
            # if loc.x < 18 and loc.x > 18.9 and loc.y > 24 and loc < 24.9:
            #     world.spawn_vehicles_straight(radius=50.0, numbers_of_vehicles=1, x=, y=)


            if time.time() - oldTime >= (59*15) and time.time() - oldTime < (59*16) or index == 12:
                    print("It has been 15 minutes, scenario ending")
                    print("index: " + str(index))
                    print(str(time.time()-oldTime/59))
                    break

            if agent.done():
                if args.loop:
                    agent.set_destination(random.choice(spawn_points).location)
                    print("searching for another target")
                else:
                    print("The target has been reached, sending new target")
                    print("\nThis is happening RIGHT NOW")
                    agent.set_destination(end_location=different_end_destinations[index])
                    index += 1

            control = agent.run_step()
            control.manual_gear_shift = False
            world.player.apply_control(control)

    finally:

        if world is not None:
            settings = world.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(True)

            world.destroy(world=world.world)

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================

def main():
    """Main method"""

    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='Print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='600x400',
        help='Window resolution (default: 1280x720)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='Actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '-l', '--loop',
        action='store_true',
        dest='loop',
        help='Sets a new random destination upon reaching the previous one (default: False)')
    argparser.add_argument(
        "-a", "--agent", type=str,
        choices=["Behavior", "Basic"],
        help="select which agent to run",
        default="Behavior")
    argparser.add_argument(
        '-b', '--behavior', type=str,
        choices=["cautious", "normal", "aggressive"],
        help='Choose one of the possible agent behaviors (default: normal) ',
        default='normal')
    argparser.add_argument(
        '-s', '--seed',
        help='Set seed for repeating executions (default: None)',
        default=None,
        type=int)

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
