## 物理引擎

爸爸：KD-Engine  爷爷：Bullet Physics

- 碰撞检测：SAT
  
  已完成的碰撞对：Box-Box Box-sphere Box-Convex Sphere-Sphere Sphere-Convex Convex-Convex

- 约束求解：Sequential Impulse

### 开发日志

- 2.26-3.3
  
  - box box碰撞跑通，引擎框架搭建完成
  
  - 破碎算法移植完成
  
  - 单线程，可以满足基本运行帧率

- 3.4-3.24
  
  - mesh mesh碰撞移植完成，待解决问题：
    
    - 穿模问题：一定程度上解决
    
    - penetration约束求解是否必要：应该是引擎里抄了两份不同版本的代码，penetration部分在现有算法里没有影响，不必纠结
  
  - 尝试一部分多线程并行化，待解决内容：
    
    - 时序一致性：已解决
    
    - 加锁问题：已解决
  
  - sequential impulse稳定性
    
    - 抖动、小程度穿模（一定程度上解决）cid：79a2a09a
  
  - 其他碰撞对：
    
    - box mesh = mesh mesh√
    - sphere shpere √
    - box shpere √
    - mesh shpere = sphere triangles√
    - cylinder cylinder = cylinder triangles
    - box cylinder = cylinder triangles
    - shpere cylinder
    - mesh cylinder = cylinder triangles

### 问题日志

- 窄域碰撞优化点：contact point cache size需要设置较大一点，实际计算时取depth最大的若干个即可，由于contact points已排过序，此时这些contact point即为最深的点

- 复杂几何体碰撞：偶尔穿模，示例为blob/0311/test4.obj
  
  - 棱碰撞问题：较薄的刚体在尖棱与其他模型发生碰撞时容易穿模，
    
    - 原因：clipFaceAgainstFace计算时所选取的witnessFace未能包含实际侵入点
    
    - 解决：findSeparateAxis找到最深点时检查是否在对面刚体内，若在则直接加入contact points
  
  - 严重问题：find separate axis的contact point计算错误
    
    - 原因：分离轴支撑点计算错误（应为物体上的点，不是在轴上的投影点）

- 多线程Release版崩溃：lambda参数捕获机制
  
  - World类多线程lambda this = &捕获成员变量出现错误，只能函数内拷贝成局部变量，然后用&传递，=和名称传递都不行，只有World类有此现象，可能是因为多线程重复访问同一个地址，编译器的优化保护机制有影响

- sequential impulse稳定性
  
  - 尝试使用各向异性的margin，没有明显改善，改回固定margin（0.005）
  - 部分解决：
    - 顾名思义，sequenetial不能用多线程（一部分解决，小碎块仍然不稳定），同时解决了时序一致性的问题
    - 设置刚体的sleep threshold（速度小于阈值超过一定时间）
    - 还是会有穿模，示例为blob/0312/world

### 效果

gif帧率10帧，实际运行帧率为60帧

![demo.gif](./screenshots/demo.gif)