# 必要项
string file_name

# 可选，当前不支持此能力。默认音频为 16k1ch
uint32 sample_rate                       

# 必要项，默认使用 INTERACTION_L6
TtsPriorityLevel priority_level 

# 可选，非产品要求勿用，优先级加权参数(0-99)，最小单位为 1
uint32 priority_weight                  

# 必要项，标识调用方来源
string domain

# 可选，用于标识播报的请求id
string trace_id

# 必要项，是否打断同等优先级播报（ true：打断；false：不打断，进入队列播报）
bool is_interrupted