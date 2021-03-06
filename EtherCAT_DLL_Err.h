#ifndef _Syntek_EtherCAT_Master_Err
#define _Syntek_EtherCAT_Master_Err

typedef enum
{
	ERR_ECAT_NO_ERROR							 = 0,
	ERR_ECAT_HW_NO_INITIALIZE,
	ERR_ECAT_HW_PWM_INITIAL,
	ERR_ECAT_HW_HAS_INITIALIZED,

	ERR_ECAT_EEPROM_READ						 = 0x10,
	ERR_ECAT_EEPROM_WRITE,
	ERR_ECAT_ENVIRONMENT_RECORD_DISABLE,
	ERR_ECAT_ENVIRONMENT_RECORD_NO_MATCH,
	ERR_ECAT_ENVIRONMENT_RECORD_FILE_OPEN,
	ERR_ECAT_ENVIRONMENT_RECORD_NOT_CREATE,
	ERR_ECAT_XML_FILE_PATH,
	ERR_ECAT_DEVICE_OPEN,
	ERR_ECAT_NO_DEVICE,
	ERR_ECAT_NO_MASTER,
	ERR_ECAT_NO_SLAVE,
	ERR_ECAT_UNKNOWN_SLAVE,
	ERR_ECAT_IST_CREATE,
	ERR_ECAT_MASTER_CREATE,
	ERR_ECAT_MASTER_REQUEST_STATE,
	ERR_ECAT_MASTER_OPERATION_NOT_READY,
	ERR_ECAT_DELTA_NODE_ID_ALIAS_READ,
	ERR_ECAT_MASTER_GET_SERIAL_NO_WRONG,
	ERR_ECAT_MASTER_GET_SERIAL_NO_TIMEOUT,

	ERR_ECAT_PIPELINE_CORE_TIMER_CREATE			= 0x80,
	ERR_ECAT_PIPELINE_CREATE,
	ERR_ECAT_COMMAND_ENQUEUE,
	ERR_ECAT_API_BUFFER_ENQUEUE,

	ERR_ECAT_NODE_ID							= 0x100,
	ERR_ECAT_SLOT_ID,
	ERR_ECAT_SDO_DOWNLOAD,
	ERR_ECAT_SDO_UPLOAD,
	ERR_ECAT_GET_PROCESS_DATA,

	ERR_ECAT_DIO_CHANNEL_INVALID				= 0x200,
	ERR_ECAT_ADDA_CHANNEL_INVALID,
	ERR_ECAT_MOTION_NOT_FINISHED,
	ERR_ECAT_SET_PULSE_MODE,
	ERR_ECAT_SET_SOFTLIMIT,
	ERR_ECAT_SET_POSITION,
	ERR_ECAT_GET_SPEED,
	ERR_ECAT_GET_MCDONE,
	ERR_ECAT_SET_HOME_CONFIG,
	ERR_ECAT_SET_P2P_CONFIG,
	ERR_ECAT_SET_PT_CONFIG,
	ERR_ECAT_SET_PV_CONFIG,
	ERR_ECAT_SET_CSP_CONFIG,
	ERR_ECAT_SET_MULTI_AXES_LINE_CONFIG,
	ERR_ECAT_SET_MULTI_AXES_ARC_CONFIG,

	ERR_ECAT_MD1_SET_GEAR						= 0x300,
	ERR_ECAT_MD1_SET_P_CHANGE,
	ERR_ECAT_MD1_SET_V_CHANGE,
	ERR_ECAT_MD1_SET_SOFTLIMIT,
	ERR_ECAT_MD1_SET_SLD,
	ERR_ECAT_MD1_SET_HOME_CONFIG,
	ERR_ECAT_MD1_SET_P2P_CONFIG,
	ERR_ECAT_MD1_SET_V_MOVE_CONFIG,
	ERR_ECAT_MD1_SET_LINE_CONFIG,
	ERR_ECAT_MD1_SET_ARC_CONFIG,

	ERR_ECAT_PATH_NOT_SUPPORT					= 0x400,
	ERR_ECAT_PATH_AXIS_NUM,
	ERR_ECAT_PATH_AXIS_NO,
	ERR_ECAT_PATH_PARA,
	ERR_ECAT_PATH_ISR_FUNC_EVENT,
	ERR_ECAT_PATH_AXISNO_UNDER_GROUP,

	ERR_ECAT_PATH_ROBOT_NOT_SUPPORT				= 0x480,
	ERR_ECAT_PATH_ROBOT_STOP,
	ERR_ECAT_PATH_ROBOT_AXIS_OVERFLOW,
	ERR_ECAT_PATH_ROBOT_BUFFER_FULL,

	ERR_ESI_INITIAL								= 0xF00,
	ERR_ESI_OPEN_DEVICE,
	ERR_ESI_CREATE_CANOPEN_OD_LIST,
	ERR_ESI_NO_DATA_TYPE_INFO,
	ERR_ESI_NO_OBJECT_INFO,
	ERR_ESI_CREATE_SYNC_MANAGER,
	ERR_ESI_CREATE_FMMU_CONTROL,
	ERR_ESI_NO_PDO_CHANNEL,
	ERR_ESI_NO_PDO_MAPPING,
	ERR_ESI_PDO_MAPPING_INSERT,
	ERR_ESI_PDO_MAPPING_DELETE,
	ERR_ESI_CREATE_DISTRIBUTED_CLOCK,

	ERR_ESI_ENI_INFORMATION_INITIAL				= 0xFF0,
	ERR_ESI_ENI_FILE_INITIAL,
	ERR_ESI_ENI_FILE_SAVE,

	ERR_ECAT_NO_SLAVE_FOUND						= 0x1000,
	ERR_ECAT_INITIAL_TIMEOUT,
	ERR_ECAT_MODE_CHANGE_FAILED,
	ERR_ECAT_SLAVE_ID,
	ERR_ECAT_ALIAS_SLAVE_ID,

	ERR_ECAT_NEED_INITIAL						= 0x1100,
	ERR_ECAT_NEED_RESET,
	ERR_ECAT_NEED_CONNECT,
	ERR_ECAT_NEED_DC_OP,
	ERR_ECAT_NEED_RALM,
	ERR_ECAT_NEED_SVON,
	ERR_ECAT_NEED_HOMECONFIG,
	ERR_ECAT_NEED_STOP,

	ERR_ECAT_RING_BUFFER_FULL					= 0x1200,
	ERR_ECAT_API_PARAMETER,
	ERR_ECAT_SLAVE_TYPE,
	ERR_ECAT_TARGET_REACHED,
	ERR_ECAT_MODE_NOT_SUPPORT,
	ERR_ECAT_MOTION_TYPE,
	ERR_ECAT_PDO_NOT_MAPPING,
	ERR_ECAT_MODULE_REVISION,
	ERR_ECAT_SPEED_CONTINUE_MODE,
	ERR_ECAT_HOME_MODE,
	ERR_ECAT_HOME_OFFSET,
	ERR_ECAT_HOME_FIRST_SPEED,
	ERR_ECAT_HOME_SECOND_SPEED,
	ERR_ECAT_HOME_ACC,
	ERR_ECAT_MRAM_INDEX,
	ERR_ECAT_MRAM_INDEX_OUT_RANGE,

	ERR_ECAT_PDO_TX_FAILED						= 0x1300,
	ERR_ECAT_SDO_TIMEOUT,
	ERR_ECAT_SDO_RETURN,
	ERR_ECAT_PDO_RX_FAILED,
	ERR_ECAT_MAILBOX,
	ERR_ECAT_SDO_BUFFER_FULL,

	ERR_ECAT_GROUP_NUMBER						= 0x1400,
	ERR_ECAT_GROUP_ENABLE,
	ERR_ECAT_GROUP_PAUSE,
	ERR_ECAT_GROUP_SLAVE,
	ERR_ECAT_GROUP_MODE,
	ERR_ECAT_GROUP_ALREADY_USED,
	ERR_ECAT_GROUP_TYPE,
	ERR_ECAT_GROUP_SVON,
	ERR_ECAT_GROUP_ALM,
	ERR_ECAT_GROUP_DATA_BUFFER,
	ERR_ECAT_GROUP_TIMEOUT,

	ERR_ECAT_SERVO_PARA_EMPTY					= 0x1500,
	ERR_ECAT_SERVO_PARA_RO,
	ERR_ECAT_SERVO_COMPARE_ENABLE,

	ERR_ECAT_RECORD_TYPE						= 0x1600,

	ERR_ECAT_MPG_ENABLE							= 0x1700,
	ERR_ECAT_MPG_CONFIG,

	ERR_ECAT_ROBOT_TYPE							= 0x2000,
	ERR_ECAT_ROBOT_INITIAL,
	ERR_ECAT_ROBOT_UPDATE,
	ERR_ECAT_ROBOT_FAILED,
	ERR_ECAT_ROBOT_RESET,
	ERR_ECAT_ROBOT_PARAMETER,
	ERR_ECAT_ROBOT_IN_MOTION,
	ERR_ECAT_ROBOT_BUFFER_FULL,
	ERR_ECAT_ROBOT_NEED_SERVO_OFF,
	ERR_ECAT_ROBOT_V_CHANGING,

	ERR_ECAT_SECURITY_OPERATING					= 0x3000,
	ERR_ECAT_SECURITY_NEED_LOGIN,
	ERR_ECAT_SECURITY_CONNECT,

	ERR_ECAT_SPLC_CONNECT_FAILED				= 0x4000,
	ERR_ECAT_SPLC_RTSS_FAILED,
	ERR_ECAT_SPLC_KERENL_FILE,
	ERR_ECAT_SPLC_ONGOING,		
	ERR_ECAT_SPLC_SDO_FAILED,		
	ERR_ECAT_SPLC_ALREADY_RUN,
	ERR_ECAT_SPLC_MOTION_CONFIG_RUN,
	ERR_ECAT_SPLC_MOT_MODE,
	ERR_ECAT_SPLC_HOMING_ERROR,
	ERR_ECAT_SPLC_MOTION_TYPE_ERROR,
	ERR_ECAT_SPLC_MOTION_ERROR,
	ERR_ECAT_SPLC_MOTION_Jog_ERROR,

	//MP相關錯誤
	ERR_ECAT_SPLC_AXISREF_STRUCT				= 0x4200,//檢查陣列大小是否正確
	ERR_ECAT_SPLC_MASTER_INDEX_NOT_FOUND,			//卡號錯誤
	ERR_ECAT_SPLC_AXIS_INDEX_NOT_FOUND,				//站號錯誤
	ERR_ECAT_SPLC_MODULE_TYPE,						//模組型態錯誤
	ERR_ECAT_SPLC_FIFO_FULL,						//輸入端資料填滿Buffer
	ERR_ECAT_SPLC_PARAM_INPUT_ERROR,				//輸入端參數錯誤
	ERR_ECAT_SPLC_AXISNUM_ERROR,					//輸入端軸數錯誤
	ERR_ECAT_SPLC_RECORD_FIFO_FULL,
	ERR_ECAT_SPLC_INVALID_MEMORY_PTR,

	//Security
	ERR_ECAT_SPLC_LICENCE_ERROR					= 0x4400,//此平台未經過授權

	//Master
	ERR_ECAT_SPLC_EVENT_FAILED					= 0x4600,
	ERR_ECAT_SPLC_SHAREMEMORY_FAILED,
	ERR_ECAT_SPLC_MASTER_NOT_FOUND,
	ERR_ECAT_SPLC_MASTER_CONNECT_FAILED,
	ERR_ECAT_SPLC_MASTER_START_SPLC_FAILED,
	ERR_ECAT_SPLC_MASTER_OPEN_eCLR_FAILED,
	ERR_ECAT_SPLC_MASTER_DISCONNECT_FAILED,
	ERR_ECAT_SPLC_MASTER_CLOSE_FAILED,

	ERR_PATH_BOARD_INIIT						= 0x8000,
	ERR_PATH_BOARD_NUMBER ,
	ERR_PATH_INITIAL_BOARD_NUMBER,
	ERR_PATH_BASE_ADDR_ERROR,
	ERR_PATH_BASE_ADDR_CONFLICT ,
	ERR_PATH_DUPLICATE_BOARD_SETTING,
	ERR_PATH_DUPLICATE_IRQ_SETTING,
	ERR_PATH_ENC_NUMBER ,
	ERR_PATH_MODULE_NUMBER,
	ERR_PATH_TIMER_VALUE,
	ERR_PATH_ENABLE,
	ERR_PATH_RANGE,
	ERR_PATH_MEMORY_ALLOC,
	ERR_PATH_MOTION_BUSY,
	ERR_PATH_MOTION_NO_START,
	ERR_PATH_WRONG_SPEED,
	ERR_PATH_WRONG_ACCTIME,
	ERR_PATH_IO_ALAM,
	ERR_PATH_OPEN_FILE_FAILED,
	ERR_PATH_MEMORY_ALLOCATE,
	ERR_PATH_MEMORY_NOT_FREE,
	ERR_PATH_OUTPUT_FILE_NOT_CLOSE,
	ERR_PATH_MOVE_AXIS_NOT_MATCH,
	ERR_PATH_PITCH_ZERO,
	ERR_PATH_TIMEOUT,
	ERR_PATH_PCI_BIOS_NOT_EXIST,
	ERR_PATH_BUFFER_FULL,
	ERR_PATH_ERROR,
	ERR_PATH_REACH_SWLIMIT,
	ERR_PATH_NO_SUPPRT_MODE,   
	ERR_PATH_AXIS_CORRELATION,
	ERR_PATH_FEEDHOLD_SUPPROT,
	ERR_PATH_SD_STOP_ON,
	ERR_PATH_VELOCITY_CHANGE_SUPER,
	ERR_PATH_COMMAND_SET,
	ERR_PATH_SDO_MESSAGE_CHOKE,
	ERR_PATH_VELOCITY_CHANGE_BUFFER_FEEDHOLD,
	ERR_PATH_VELOCITY_CHANGE_SYNC_MOVE,
	ERR_PATH_VELOCITY_CHANGE_SD_ON,
	ERR_PATH_POS_CHANGE_MODE,
	ERR_PATH_BUFFER_LENGTH,
	ERR_PATH_2SEG_DISTANCE,
	ERR_PATH_ARC_CENTER_POSITION,
	ERR_PATH_ARC_END_POSITION,
	ERR_PATH_ARC_ANGLE_CALC,
	ERR_PATH_ARC_RADIUS_CALC,
	ERR_PATH_GEAR_SETTING,
	ERR_PATH_CAM_TABLE,
	ERR_PATH_AXES_NUMBER,
	ERR_PATH_SPIRAL_END_POSITION,
	ERR_PATH_SPEED_MODE_SLAVE,
	ERR_PATH_SPEED_MODE_SET_SLAVE,
	ERR_PATH_VELOCITY_CHANGE, 
	ERR_PATH_BACKLASH_STEP, 
	ERR_PATH_BACKLASH_STATUS,
	ERR_PATH_DIST_OVER,
	ERR_PATH_ECAT_DLL_ERROR_CODE,
	ERR_PATH_ECAT_NEED_ENABLE,
	ERR_PATH_ECAT_ECAM_ENABLE,
	ERR_PATH_ECAT_ECAM_MASTERSOURCE,

	ERR_RTX_RTSS_LOAD							= 0xD000,
	ERR_RTX_CONNECT_LINK_FAILED,
	ERR_RTX_EVENT_FAILED,
	ERR_RTX_CONNECT_FAILED,
	ERR_RTX_CONFIG_EDITED,
	ERR_RTX_SECURITY_FAILED,
	ERR_RTX_COMMANDING,
	ERR_RTX_RTSS_SYSTEM_NOT_SUPPORT,
	ERR_RTX_NOT_SUPPORT,
	ERR_RTX_THREAD_CREATE_FAILED,
	ERR_RTX_RTSS_START_FAILED,

	ERR_RTX_WIN32_SYSTEM_NOT_SUPPORT			= 0xD100,
	ERR_RTX_CALLBACK_CLOSE,
	ERR_RTX_CALLBACK_FUNCTION,
	ERR_RTX_CALLBACK_THREAD,

	ERR_RTX_ERRORLOG_NOT_ENABLE					= 0xD200,
	ERR_RTX_ERRORLOG_COUNT_ERROR,

	ERR_CARD_NO_FOUND							= 0xE000,
	ERR_CARD_NO_RESPONSE,
	ERR_CARD_CONNECT_FAILED,
	ERR_CARD_MEMORY_NOT_ENOUGH,
	ERR_CARD_LOAD_AUTOCONFIG_FILE,
	ERR_CARD_SECURITY_FAILED,
	ERR_CARD_UPGRADE_CREATE_THREAD_FAILED,
	ERR_CARD_UPGRADE_NO_RESPONSE,
	ERR_CARD_UPGRADE_NO_RESOURSE,
	ERR_CARD_UPGRADE_LOAD_RESOURSE,
	ERR_CARD_UPGRADE_TIMEOUT,
	ERR_CARD_UPGRADE_FAILED,

	ERR_ECAT_DLL_IS_USED						= 0xF000,
	ERR_ECAT_NO_DLL_FOUND,
	ERR_ECAT_NO_RTSS_DLL_FOUND,
	ERR_ECAT_NO_CARD_DLL_FOUND,
	ERR_ECAT_NO_ESI_DLL_FOUND,
	ERR_ECAT_SAME_CARD_NUMBER,
	ERR_ECAT_CARDNO_ERROR,
	ERR_ECAT_GET_DLL_PATH,
	ERR_ECAT_GET_DLL_VERSION,
	ERR_ECAT_NOT_SUPPORT,

	ERR_ECAT_LOADLIB_EMPTY						= 0xFFFF
}E_ECAT_ERROR_CODE;


#endif