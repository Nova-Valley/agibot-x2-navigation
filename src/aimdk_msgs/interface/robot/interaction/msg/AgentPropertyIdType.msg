uint8 value

uint8 AGENT_PROPERTY_UNDEFINED = 0x00

# value: "zh/en"
uint8 AGENT_PROPERTY_LANGUAGE = 0x01 

# 运行模式 value: "normal/only_voice/no_voice" normal: 正常模式，启用语音助手和语音识别  only_voice: 只启用提供降噪后音频  no_voice: 完全禁用agent，音频都不进行获取提供
uint8 AGENT_PROPERTY_RUN_MODE = 0x02 

# value: true/false
uint8 AGENT_PROPERTY_GREET = 0x03 

# value: true/false     
uint8 AGENT_PROPERTY_HEAD_FOLLOW = 0x04  

# value: "60" 超时时间，单位秒
uint8  AGENT_PROPERTY_DUPLEX_TIMEOUT = 0x05

# value: "环境切换指令"
uint8  AGENT_PROPERTY_ENVIRONMENT_SWITCH = 0x06

# value: "人脸迎宾"
uint8  AGENT_PROPERTY_WELCOME_STATUS = 0x07