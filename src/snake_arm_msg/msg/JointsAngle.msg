# 整个机械臂的关节角描述
# 关于数组的使用，可以参考
# https://blog.csdn.net/qq_40976407/article/details/103531657

Header header
uint32 JointsNumber   # 关节个数
JointAngle[] Joints   # 关节角数组，0对应基部关节，数组大小应与JointsNumber保持一致。单位：弧度