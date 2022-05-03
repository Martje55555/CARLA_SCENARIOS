from __future__ import print_function

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import ColorConverter as cc

import argparse
import logging
import math
import random
import re
import weakref
import time

IM_WIDTH = 2560
IM_HEIGHT = 1440
actor_list = []
all_id = []
walkers_list = []
normal_vehicles = []

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q

except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

# Find the weather presets to be able to use in environment
def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

# Return the different actor names
def get_actor_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

# Return the different actor blueprints
def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

# Spawning Pedestrians givent the world, client and amount of pedestrians
def spawn_pedestrians(world, client, number_of_pedestrians):
    # add pedestrians to the world
    blueprintsWalkers = world.get_blueprint_library().filter("walker.pedestrian.*")
    walker_bp = random.choice(blueprintsWalkers)

    spawn_points = []
    for i in range(number_of_pedestrians):
        spawn_point = carla.Transform()
        spawn_point.location = world.get_random_location_from_navigation()
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

    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
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
 
    print(len(walkers_list))


class World(object):
    def __init__(self, carla_world, args, client):
        self.world = carla_world
        self.sync = args.sync
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.player = None
        self.agro_vehicles = []
        self.cautious_vehicles = []
        self.normal_vehicles = []
        self.collision_sensor = None
        self.camera_manager = None
        self.client = client
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        self.restart()
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.current_map_layer = 0
        # traffic manager for normal vehicles
        self.tm1 = self.client.get_trafficmanager()
        self.tm1.set_random_device_seed(9)
        self.tm1.set_respawn_dormant_vehicles(True)
        self.tm1.set_boundaries_respawn_dormant_vehicles(20, 500)
        # traffic manager for cautius vehicles
        self.tm2 = self.client.get_trafficmanager()
        self.tm2.set_random_device_seed(9)
        self.tm2.set_respawn_dormant_vehicles(True)
        self.tm2.set_boundaries_respawn_dormant_vehicles(20, 500)
        # traffic mangaer for aggressiv vehicles
        self.tm3 = self.client.get_trafficmanager()
        self.tm3.set_random_device_seed(9)
        self.tm3.set_respawn_dormant_vehicles(True)
        self.tm3.set_boundaries_respawn_dormant_vehicles(20, 500)

        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]

    def restart(self):
        # self.player_max_speed = 1.589
        # self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0

        # get tesla vehicle from blueprint
        blueprint_library = self.world.get_blueprint_library()
        bp = blueprint_library.filter('model3')[0]

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(bp, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
            self.player.set_autopilot(True)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(bp, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
            self.player.set_autopilot(True)
        
        # spawn more vehicles
        #spawn_points = self.map.get_spawn_points()
        #self.spawn_vehicles_around_ego_vehicles(radius=50, numbers_of_vehicles=15)

        # spawn pedestrians
        #spawn_points = self.map.get_spawn_points()
        #spawn_pedestrians(world=self.world, client=self.client, number_of_pedestrians=80)

        # Set up the sensors.
        self.camera_manager = CameraManager(self.player, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    # Function to spawn "normal" vehicles around the ego or hero vehicle
    def spawn_vehicles_around_ego_vehicles(self, radius, numbers_of_vehicles):
        spawn_points = self.map.get_spawn_points()
        np.random.shuffle(spawn_points)  # shuffle  all the spawn points
        ego_location = self.player.get_location()
        accessible_points = []
        for spawn_point in spawn_points:
            dis = math.sqrt((ego_location.x-spawn_point.location.x)**2 + (ego_location.y-spawn_point.location.y)**2)
            # it also can include z-coordinate,but it is unnecessary
            if dis < radius:
                print(dis)
                accessible_points.append(spawn_point)

        vehicle_bps = self.world.get_blueprint_library().filter('vehicle.*.*')   # don't specify the type of vehicle
        vehicle_bps = [x for x in vehicle_bps if int(x.get_attribute('number_of_wheels')) == 4]  # only choose car with 4

        vehicle_list = []  # keep the spawned vehicle in vehicle_list, because we need to link them with traffic_manager
        if len(accessible_points) < numbers_of_vehicles:
            # if your radius is relatively small,the satisfied points may be insufficient
            numbers_of_vehicles = len(accessible_points)

        for i in range(numbers_of_vehicles):
            point = accessible_points[i]
            vehicle_bp = np.random.choice(vehicle_bps)
            try:
                vehicle = self.world.spawn_actor(vehicle_bp, point)
                vehicle_list.append(vehicle)
                #actor_list.append(vehicle)
                self.normal_vehicles.append(vehicle)
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
        print(len(vehicle_list))

    # Similar to the previous function, the only difference is that we are changing some parameters to reflect aggressive vehicles
    def spawn_agro_vehicles(self, radius, numbers_of_vehicles, update):
        if update:
            print('THIS IS ACTUALLY HAPPENING')
            tm_port = self.tm3.get_port()  # get the port of tm. we need add vehicle to tm by this port
            print(len(self.agro_vehicles))
            for v in self.agro_vehicles:  # set every vehicle's mode
                self.tm3.ignore_lights_percentage(v, 0)
                self.tm3.distance_to_leading_vehicle(v, 5.0)
                self.tm3.vehicle_percentage_speed_difference(v, -5)
                self.tm3.update_vehicle_lights(v, True)
        else:
            spawn_points = self.map.get_spawn_points()
            np.random.shuffle(spawn_points)  # shuffle  all the spawn points
            ego_location = self.player.get_location()
            accessible_points = []
            for spawn_point in spawn_points:
                dis = math.sqrt((ego_location.x-spawn_point.location.x)**2 + (ego_location.y-spawn_point.location.y)**2)
                if dis < radius:
                    print(dis)
                    accessible_points.append(spawn_point)

            vehicle_bps = self.world.get_blueprint_library().filter('vehicle.*.*')   # don't specify the type of vehicle
            vehicle_bps = [x for x in vehicle_bps if int(x.get_attribute('number_of_wheels')) == 4]  # only choose car with 4

            vehicle_list = []  # keep the spawned vehicle in vehicle_list, because we need to link them with traffic_manager
            if len(accessible_points) < numbers_of_vehicles:
                # if your radius is relatively small,the satisfied points may be insufficient
                numbers_of_vehicles = len(accessible_points)

            for i in range(numbers_of_vehicles):  # generate the free vehicle
                point = accessible_points[i]
                vehicle_bp = np.random.choice(vehicle_bps)
                try:
                    vehicle = self.world.spawn_actor(vehicle_bp, point)
                    vehicle_list.append(vehicle)
                    #actor_list.append(vehicle)
                    self.agro_vehicles.append(vehicle)
                except:
                    print('failed')  # if failed, print the hints.
                    
            self.tm3.global_percentage_speed_difference(10.0)  # set the global speed limitation
            tm_port = self.tm3.get_port()
            for v in vehicle_list:  # set every vehicle's mode to be "aggressive"
                v.set_autopilot(True, tm_port)
                self.tm3.ignore_lights_percentage(v, 100)
                self.tm3.distance_to_leading_vehicle(v, 0)
                self.tm3.vehicle_percentage_speed_difference(v, -20)
                self.tm3.update_vehicle_lights(v, True)
            print(len(vehicle_list))

    # Similar to the previous function, the only difference is that we are changing some parameters to reflect cautious vehicles
    def spawn_cautious_vehicles(self, radius, numbers_of_vehicles, update):

        if update == True:
            print('THIS IS ACTUALLY HAPPENING')
            tm_port = self.tm2.get_port()  # get the port of tm. we need add vehicle to tm by this port
            print(len(self.cautious_vehicles))
            for v in self.cautious_vehicles:  # set every vehicle's mode
                self.tm2.ignore_lights_percentage(v, 0)
                self.tm2.distance_to_leading_vehicle(v, 5.0)
                self.tm2.vehicle_percentage_speed_difference(v, -5)
                self.tm2.update_vehicle_lights(v, True)
        else:
            spawn_points = self.map.get_spawn_points()
            np.random.shuffle(spawn_points)  # shuffle  all the spawn points
            ego_location = self.player.get_location()
            accessible_points = []
            for spawn_point in spawn_points:
                dis = math.sqrt((ego_location.x-spawn_point.location.x)**2 + (ego_location.y-spawn_point.location.y)**2)
                if dis < radius:
                    print(dis)
                    accessible_points.append(spawn_point)

            vehicle_bps = self.world.get_blueprint_library().filter('vehicle.*.*')   # don't specify the type of vehicle
            vehicle_bps = [x for x in vehicle_bps if int(x.get_attribute('number_of_wheels')) == 4]  # only choose car with 4

            vehicle_list = []  # keep the spawned vehicle in vehicle_list, because we need to link them with traffic_manager
            if len(accessible_points) < numbers_of_vehicles:
                # if your radius is relatively small,the satisfied points may be insufficient
                numbers_of_vehicles = len(accessible_points)

            for i in range(numbers_of_vehicles):  # generate the free vehicle
                point = accessible_points[i]
                vehicle_bp = np.random.choice(vehicle_bps)
                try:
                    vehicle = self.world.spawn_actor(vehicle_bp, point)
                    vehicle_list.append(vehicle)
                    self.cautious_vehicles.append(vehicle)
                except:
                    print('failed')  # if failed, print the hints.
            
            #tm.global_percentage_speed_difference(10.0)  # set the global speed limitation
            tm_port = self.tm2.get_port()  # get the port of tm. we need add vehicle to tm by this port
            for v in vehicle_list:  # set every vehicle's mode to reflect a cautious vehicle
                v.set_autopilot(True, tm_port)
                self.tm2.distance_to_leading_vehicle(v, 10.0)
                self.tm2.vehicle_percentage_speed_difference(v, 60)
                self.tm2.update_vehicle_lights(v, True)
                self.tm2.auto_lane_change(v, False)
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
        self.camera_manager.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        sensors = [
            self.camera_manager.sensor,]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()

    def destroyVehicles(self, typeOfCars):
        if typeOfCars == 'agro':
            for vehicle in self.agro_vehicles:
                vehicle.destroy()

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================

class CameraManager(object):
    def __init__(self, parent_actor, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.spawn_point = carla.Transform(carla.Location(x=.20, y=0, z=1.10))

        self.transform_index = 1

        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
        ]

        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            bp.set_attribute('fov', '110')
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(IM_WIDTH))
                bp.set_attribute('image_size_y', str(IM_HEIGHT))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)

            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, force_respawn=True)

    def set_sensor(self, index, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self.spawn_point,
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        
        self.index = index

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self.sensors[self.index][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================

class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0

    def parse_events(self, world):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(20.0)

        sim_world = client.get_world()
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        if args.autopilot and not sim_world.get_settings().synchronous_mode:
            print("WARNING: You are currently in asynchronous mode and could "
                  "experience some issues with the traffic simulation")

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()

        world = World(sim_world, args, client)

        controller = KeyboardControl(world, True)

        static_weather_parameters = [
            carla.WeatherParameters.ClearNoon,  #0
            carla.WeatherParameters.CloudyNoon, #1
            carla.WeatherParameters.WetNoon,     #2
            carla.WeatherParameters.WetCloudyNoon, #3 
            carla.WeatherParameters.MidRainyNoon,  #4
            carla.WeatherParameters.HardRainNoon,  #5
            carla.WeatherParameters.SoftRainNoon,  #6 
            carla.WeatherParameters.ClearSunset,   #7
            carla.WeatherParameters.CloudySunset,  #8
            carla.WeatherParameters.WetSunset,     #9
            carla.WeatherParameters.WetCloudySunset, #10
            carla.WeatherParameters.MidRainSunset,    #11
            carla.WeatherParameters.HardRainSunset,   #12
            carla.WeatherParameters.SoftRainSunset,   #13
        ]

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()

        oldTime = time.time()
        weather = static_weather_parameters[0] # Clear Noon

        while True:
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)
            if controller.parse_events(world):
                return

            # Hard Rain Sunset - 1 min marker
            if time.time() - oldTime >= (10) and time.time() - oldTime < (20) and weather != static_weather_parameters[12]:
                weather = static_weather_parameters[12]
                sim_world.set_weather(weather)
                world.spawn_vehicles_around_ego_vehicles(radius=50, numbers_of_vehicles=10)
                
            # Clear Noon - 2 min marker
            if time.time() - oldTime >= (20) and time.time() - oldTime < (30) and weather != static_weather_parameters[0]:
                weather = static_weather_parameters[0]
                sim_world.set_weather(weather)
                world.spawn_cautious_vehicles(radius=50, numbers_of_vehicles=8, update=True)
                
            # Hard Rain Noon - 3 min marker
            if time.time() - oldTime >= (30) and time.time() - oldTime < (40) and weather != static_weather_parameters[5]:
                weather = static_weather_parameters[5]
                sim_world.set_weather(weather)
                
            # Cloudy Sunset - 4 min marker
            if time.time() - oldTime >= (40) and time.time() - oldTime < (50) and weather != static_weather_parameters[8]:
                weather = static_weather_parameters[8]
                sim_world.set_weather(weather)
                world.spawn_agro_vehicles(radius=50, numbers_of_vehicles=1, update=True)

            # Soft Rain Noon - 5 min marker
            if time.time() - oldTime >= (50) and time.time() - oldTime < (60) and weather != static_weather_parameters[6]:
                weather = static_weather_parameters[6]
                sim_world.set_weather(weather)

            world.render(display)
            pygame.display.flip()
          
    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
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
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='2560x1440',
        help='window resolution (default: 2560x1440)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "All")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    print(args)

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

    finally:
        for actor in actor_list:
            actor.destroy()

if __name__ == '__main__':

    main()