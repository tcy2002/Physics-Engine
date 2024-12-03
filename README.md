## 刚体物理引擎（rigidbody dynamics）

- 参考bullet physics，kd engine
  
  - 重写了引擎框架：宽域+窄域+约束（主流刚体物理引擎结构）
  
  - 重写了大部分物理算法：基本碰撞对、射线检测、车辆封装

- 碰撞检测：SAT

- 约束求解：Sequential Impulse

- 毁伤算法：Voronoi剖分

- 车辆/坦克：射线/碰撞

- 构建：
  
  - win：cmake生成visual studio工程，打开.sln，编译，然后将SimpleViewer/lib中的freeglut.dll和glew32.dll放入可执行文件路径下（bin/Release(Debug))
  
  - linux：cmake直接构建、编译

- Demo：
  
  - PEDemoBomb/PEDemoFracture/PEDemoCar/PEDemoTank/PEDemoIntegrated
    按x键开始物理模拟（区分大小写），摄像机视角操作方式与UE类似
  
  - PEDemoCar/PEDemoTank的操作方法请参考examples/car_demo和examples/tank_demo内的注释

- 后续开发

  - 破碎优化、集成

  - 场景设计和搭建

  - 4.13 支持linux下编译运行

  - 4.14 修了Pool的bug（疑似），支持显示Debug Points（单元测试）

  - 10.9 修复sphere-cylinder碰撞的bug

  - 10.12 增加复合刚体，修复contact vehicle的bug

  - 10.14 使用batched-loop多线程，否则由于单个task开销较小，大部分情况下并没有提升效率
    - sequential impulse不并行化的情况下，能达到kd-engine使用taskflow的帧率水平（40+）
    - sequential impulse并行化的情况下，帧率提升明显，bomb demo能跑满60帧，约束求解不再是单一的瓶颈。但是sequential impulse并行化会破坏计算稳定性（可以用染色法解决）
  
  - 11.2 concave碰撞完成（加bvh逐面片求解）

  - 11.3 配置文件load/save完成

  - 11.7 机械臂demo

  - 11.11 支持导出gltf格式; ball-joint

  - 11.17 convex-cylinder碰撞：面片vs圆柱体

  - 12.3 将线性几何工具换成eigen

- TODO:

  - 复杂约束（hinge，slider等）

  - broadphase加速: 八叉树，hash grid等

  - capsule相关碰撞

### 效果

- 复合刚体

![compound.gif](./screenshots/compound.gif)

- 凹体碰撞

![concave.gif](./screenshots/concave.gif)

- 履带坦克
  
  - DefaultRaycastSolver：根据射线检测的深度设置impulse和suspension，表现为直接贴地
  - Contact：根据车轮与地面的碰撞结果设置impulse和suspension，力的反馈更真实
  - 目前履带没有碰撞效果，会有穿模发生，正在寻找方案解决(3.25)

- 坦克综合效果

![tank](./screenshots/tank.gif)

- 破碎
  
  - 基于Voronoi的实时毁伤

![fracture.gif](./screenshots/fracture.gif)

- 城镇坦克对战场景
  
  - 建筑物实时毁伤，坦克发射的炮弹会摧毁建筑物

![urban.gif](./screenshots/urban.gif)

- 机械臂

![arm.gif](./screenshots/arm.gif)

- 导出gltf在win自带3D Viewer中的效果

![gltf](./screenshots/gltf.gif)

- terrain demo

![terrain](./screenshots/tank_terrain.gif)

### 开发日志

- 2.26-3.3
  
  - 引擎框架搭建完成，box box碰撞跑通（bullet，sat）
  
  - 破碎算法完成
  
  - 单线程，可以满足基本运行帧率

- 3.4-3.10
  
  - mesh mesh碰撞完成，待解决问题：
    
    - 穿模：一定程度上解决
    - penetration约束求解是否必要：应该是KD引擎里用了两份不同版本的算法，penetration部分在现有算法里没有影响，不必纠结
  
  - 多线程加速，待解决内容：
    
    - 时序一致性：已解决
    - 加锁问题：已解决

- 3.11-3.17
  
  - sequential impulse稳定性
    
    - 抖动、小程度穿模（一定程度上解决）cid：79a2a09a
  
  - 其他碰撞对：
    
    - box mesh = mesh mesh √
    - sphere shpere √
    - box shpere √
    - mesh shpere = sphere triangles √
    - cylinder cylinder = mesh mesh √
    - box cylinder = mesh mesh √
    - shpere cylinder √
    - mesh cylinder = mesh mesh √

- 3.18-3.24
  
  - 内存优化
    
    - FrictionContactConstraint对象占用空间较多，每一帧反复创建销毁，适合使用内存池，其他部分待定
  
  - Clion使用vs编译
    
    - 语法检查更严格
  
  - 检查碰撞检测
    
    - 1、PE_TEST_GROUND_MARGIN
      - box-sphere取消getSphereDistance的margin
      - sphere-convex取消radiusWithThreshold的margin
    - 2、碰撞对顺序
      - 涉及cylinder的全部将cylinder放在前面
      - 最近点需要统一为物体B上的点
  
  - 履带坦克
    
    - 射线检测异常：
      - 地面fixedBody须设置质量和惯性矩阵均为0
      - 轮子添加实体之后，须设置raycastExcludeIds列表
      - 履带片不宜添加实体，否则会对非kinematic地面产生错误impulse
    - 优化点：
      - 1、使用多个射线检测（目前aabb检测不支持这种方式）
      - 2、使用碰撞检测代替射线检测（已选）
        - v0.1(10012d6): 车轮会抖动(1)，行进时suspension会异常变短(2)，过坎时计算出错(3)
        - v0.2(e25dbca): (3)已解决，(2)进一步发现有两种情况，一是以地面法矢量为接触法线，二是以车轮法矢量为接触法线，前者车轮下陷，后者suspension缩短
        - v0.3(ebceaaa): (1)部分解决，仍有轻微抖动，解决方式为当没有接触发生时，按递增方式还原suspension；(2)待解决
        - v1.0(f1e366e): 问题都已初步解决，待优化的点：轮子与可活动物体的碰撞
        - v1.1(de8fb1b): 解决了溜坡的问题
        - v2.0(b399d0b): 一些细节问题

- 3.25-3.31
  
  - 射线检测补全：已完成
  
  - 移植大地形：暂时弃用
  
  - 其他优化
    
    - hash vector: 使用STL unordered_multimap
    - contact result: 也使用pool管理
  
  - simulator
    
    - 仿真器类，用于管理整个物理引擎，负责后台管理物理引擎和前台渲染，用户只需要重写init和step函数即可
    - 已完成

### 问题日志

- 窄域碰撞优化：contact point cache size需要设置较大一点，实际计算时取depth最大的若干个即可，由于contact points已排过序，此时这些contact point即为最深的点

- 复杂几何体碰撞：偶尔穿模，示例为blob/0311/test4.obj
  
  - 棱碰撞问题：较薄的刚体在尖棱与其他模型发生碰撞时容易穿模，
    - 原因：clipFaceAgainstFace计算时所选取的witnessFace未能包含实际侵入点
    - 解决：findSeparateAxis找到最深点时检查是否在对面刚体内，若在则直接加入contact points
  - 严重问题：find separate axis的contact point计算错误
    - 原因：分离轴支撑点计算错误（应为物体上的点，不是在轴上的投影点）

- 多线程Release版崩溃：lambda参数捕获机制
  
  - 多线程lambda this = &捕获成员变量出现错误，只能函数内拷贝成局部变量，然后用&传递，=和名称传递都不行，可能是因为多线程重复访问同一个地址，编译器的优化保护机制有影响
  - 3.18发现：this捕获改成auto c = this, &c则可以正常运行，原因暂不清楚

- sequential impulse稳定性
  
  - 尝试使用各向异性的margin，没有明显改善，改回固定margin（0.005）
  - 部分解决：
    - 顾名思义，sequential impulse的迭代部分不能用多线程（一部分解决，小碎块仍然不稳定），同时解决了时序一致性的问题
    - 设置刚体的sleep threshold（速度小于阈值超过一定时间）
    - 还是会有穿模，示例为blob/0312/world

- cylinder碰撞不稳定
  
  - 原因：default mesh错误，已修改

- toolset更改为vs之后编译问题：无法写入清单文件
  
  - 解决：修改cmake generator为let cmake decide

- 内存池启动崩溃
  
  - 原因：设置的块大小过小，应至少大于一个对象的大小
  - 解决：加大小判断

- contact vehicle
  
  - 问题1：车轮抖动
    - 原因：车轮与地面的接触点不稳定，没有接触点时suspension直接还原
    - 解决：改为递增还原
  - 问题2：suspension速度快时会变短
    - 原因：速度增加导致contact depth变大，contact vehicle的特殊计算方式允许较短的suspension连续存在
    - 解决：设置suspension时不考虑contact depth，但计算形变量时加入contact depth
  - 问题3：过坎时出现计算异常
    - 原因：sqrt(<0)导致NaN (contact_vehicle.cpp: 182)
  - 问题4（新问题）：刹车异常，即使坡度很小也会溜
    - 已修改车轮刹车摩擦的计算方式 (contact_vehicle.cpp: 563)

- 引擎问题：工作中出现nan
  
  - 原因：normalize(0 0 0)导致NaN (sphere_cylinder_collision_algorithm.cpp: 39)
  - 解决：37行加r>0的判断

- 碰撞问题：
  
  - 仍然存在稳定性问题
  - 碰撞时小概率发生异常impulse，目前只观察到box-cylinder存在这种情况

- 性能：
  
  - 相比KD-Engine，BombDemo的帧率明显偏低（KD-Engine 45帧左右，本引擎 24帧）
  - 将contact point max size调整为与KD-Engine相同，帧率提升到35帧左右