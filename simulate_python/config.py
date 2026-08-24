ROBOT = "h1_2" # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1"
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml" # Robot scene
DOMAIN_ID = 0 # Domain id
INTERFACE = "lo" # Interface 

USE_JOYSTICK = 1 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = True # Virtual spring band (molla verticale): sostiene il torso
# lasciandolo LIBERO di ruotare, cosi la policy fa equilibrio mentre e' appeso.
# Sostituisce il <weld> rigido (commentato in h1_2_handless.xml), che bloccava tutti
# i 6 DOF del torso e portava la policy in uno stato sbagliato (torso indietro).
# Tasti sulla finestra MuJoCo: '8' allunga la molla (scarica peso sui piedi),
# '7' la accorcia, '9' on/off. Rilascio GRADUALE col tasto '8' dopo aver attivato la policy.

SIMULATE_DT = 0.005  # Need to be larger than the runtime of viewer.sync()
VIEWER_DT = 0.02  # 50 fps for viewer

# Ground truth logger: salva lo stato VERO della base (qpos/qvel del free joint)
# in CSV, per confrontarlo coi log del controller di deploy (stima FK, IMU).
# Allineamento temporale: colonna 'time' = clock MuJoCo = LowState.tick/1000.
ENABLE_STATE_LOG = True
STATE_LOG_DIR = "../../deploy/logs"  # accanto ai run del deploy (deploy/logs/sim_<ts>/)
STATE_LOG_DT = 0.01                  # 100 Hz
