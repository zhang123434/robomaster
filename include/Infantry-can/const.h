#ifndef CONST
#define CONST

enum COLOR{
    BLUE = 1,
    RED = 2,
    OTHER_COLOR = 3
};

enum ArmorType{
    SMALL = 0,
    BIG = 1,
};

/*enum ArmorState{
    LOST = 0,       // 丢失目标
    FIRST = 1,      // 第一次发现目标
    SHOOT = 2       // 持续识别目标
};*/
enum ArmorState{
   //没识别到不发子弹
   NO_FIND_NO_SHOOT=0,
   //识别到不发射子弹
   FIND_NO_SHOOT=1,
   //识别到且发射子弹
   FIND_SHOOT=2,
   //没解算完，与上次指令相同
   NO_END_SOLVER=3,
};

enum Mode{
    NORMAL = 1,     // 普通模式
    RUNE = 2        // 打符模式
};

#endif