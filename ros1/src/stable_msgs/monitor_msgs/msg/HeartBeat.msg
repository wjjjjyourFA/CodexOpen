std_msgs/Header header
uint32 message_num

string name      # 程序名
bool is_active   # 表示是否激活
bool is_ready    # 表示是否准备好 
uint8 mode       # 操作模式 0: 默认, 1: 高优先级, 2: 低优先级
