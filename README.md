# Particle-System
1102_CA&CE Class HW1

## Result Overview
<img width="1282" alt="截圖 2022-05-16 下午11 12 33" src="https://user-images.githubusercontent.com/84212529/168625561-a361f0d1-aaa8-457b-9cdd-f875ee9e2a69.png">

<img width="1283" alt="截圖 2022-05-15 下午4 06 42" src="https://user-images.githubusercontent.com/84212529/168463357-3b8e6fa0-9990-4c34-b717-5ce2fc715657.png">

## Introduction  
透過實作不同class內之function了解粒子系統的運作方式與模式

每個粒子都擁有各自的屬性 ex: mass, position, velocity, acceleration等等
透過一次一次的模擬step去計算出下一幀的粒子狀態並且更新
更新過後的粒子狀態又會接著影響下一輪的模擬計算

而一群一群的粒子之間彼此交互運作之下就可以模擬如煙雲水流或是這次作業的布料也是由大量的粒子彼此之間互相做綁定互動與影響

## Fundamentals  
###	物件導向等概念
此次作業由大量的class所組成其中不乏有繼承等關係存在
在有物件導向的基礎之下來對這些東西進行操作會比較有感覺

###	多種型態的積分求解方式
-	ExplictEuler
-	implicitEuler
-	MidpointEuler
-	RungeKuttaFourth<br>
上面的積分器要上課 不然不知道在幹嘛 所謂的要用未來的速度來更新現在的位置還蠻饒口跟容易搞不清楚在幹嘛 我也是花了蠻多時間才理解整體概念

###	物理的基礎知識
-	彈力
-	阻尼力
-	彈性碰撞<br>
有上面的基本概念在處理物理公式的時候才容易理解

###	線性代數
- 向量操作 (內積/投影/法向量/單位法向量)<br>
向量的操作要一點概念 尤其是方向的問題要搞清楚 蠻多同學碰撞沒處理好 感覺是因為在內積Normal時方向搞錯而導致正負錯誤導致

## Implementation 
### initializeSpring() :
```
需建構粒子之間的連線 三種Type的建立邏輯
其實都跟著叫提示的 STRUCTURAL Horizontal類似
首先建立STRUCTURAL Vertical部分 只需要注意要連接的index就可以成功建立

同理在建構 BEND 以及SHEAR時 也仿效STRUCTURAL
將不同的方向 (水平/垂直) (右下/左下) 拆開成兩個loop各自連接
BEND: 除注意應連接的index之外 透過position差取得距離向量
距離向量作norm()就可以得到長度
 
SHEAR: 邏輯相同 第一個Loop做左上右下 第二Loop做右上左下
```

### computeSpringForce()
``` 
欲對每一條String的兩端點粒子計算受力 透過 a = F / m 更新各粒子加速度
實作上流程如上圖註解
(1) 針對每條string取得一些必要資訊 (ex: index , spring.length() )
(2) 套用如註解所述公式 各別計算兩粒子的彈力以及阻尼力
(3) 計算出受到的力之後 就可以透過 a = F / m 更新加速度
```
### collide( Cloth* cloth)
```
球與布粒子的碰撞
針對每一顆球以及布上面的每一個粒子 兩兩之間都要進行碰撞的檢查

Line139: 
兩粒子間向量相減後取norm() 並判斷距離是否 <= 球體半徑

Line141~146: 
修正穿模，將穿透的距離重新加回去 (需特別注意normal方向)
(normal方向為 球體粒子位置向量 – 布料粒子位置向量 = 布料指向球心方向)

Line150~154:
計算碰撞時的速度只要計算碰撞方向維度的速度 (sp_v_normal , cl_v_normal)
此數值透過 速度內積normal可以取得
Tangent(碰撞平面)的向量可透過 速度向量減去Normal方向的速度分量取得

Line156~164:
簡單的公式解並且更新各別速度即可 
(速度由碰撞後的normal方向速度與tangent速度所組合而成)
```

### collide( )
```
與布料碰撞完成之後這部分 ctrl+c ctrl+v 就完成了
不同的地方在求取兩球之間的距離時需要多加入另一球半徑
已經算過的球就不用再算過一次 邏輯蠻單純的 
第二層迴圈從第一層迴圈的index開始即可 for(int i=0) for(int j=i)
(ps 記得排除 i==j的情況以免計算到自己跟自己的碰撞)
```

### Integrator – ExplicitEuler
```
積分器的實作主要目的是為了在積分器裡面更新
(1) 粒子(衣服/球體)的下一個position
(2) 粒子(衣服/球體)的下一個velocity
 
ExplicitEuler相對簡單 根據公式分別更新各自的位置與速度即可
```

### Integrator – MidPointEuler 
```
與ExplicitEuler類似 差別是在更新下一個position以及velocity時
是使用未來的velocity以及acceleration來進行更新
所以需呼叫 SimulateOneStep取得未來的velocity / acceleration狀態

此時需要釐清整體工作流程以及SimulateOneStep的邏輯會比較容易理解
Working Flow
for每秒frame數量{
	SimulateOneStep();
積分粒子 (衣服粒子/球體粒子);
}
SimulateOneStep:
(1) 更新 衣服粒子加速度 (重力) 
(2) 更新 球體粒子加速度 (重力)
(3) 更新 衣服粒子加速度 (彈力/阻力)  
=> 會使用到粒子當下位置資訊以及速度資訊
1. 利用當下位置計算彈力 => 更新加速度
2. 利用當下速度計算阻力 => 更新加速度

(4) 更新 衣服粒子/球體粒子速度 (若衣服球體碰撞)
=> 會使用到粒子當下位置資訊以及速度資訊
1. 利用當下位置判斷是否碰撞
2. 利用當下速度計算碰撞之後的新速度

(5) 更新 球體粒子速度 (若球體間碰撞)
=> 會使用到粒子當下位置資訊以及速度資訊
1. 利用當下位置判斷是否碰撞
2. 利用當下速度計算碰撞之後的新速度

結論：呼叫SimulateOneStep的目的是為了
(1 ) 更新粒子的 加速度
(2) 更新粒子的 速度 (若碰撞產生)

更新加速度以及速度都是依照呼叫時的particle.position() particle.velocity()計算
意思就是在我們呼叫SimulateOneStep取得未來加速度以及速度時的前提
是我們必須先將position()還有velocity()更新到未來的狀況
而更新的方式就是比照ExplicitEuler更新到 deltaTime/2的地方
 
0.先將目前衣服以及球體的三個資訊保存下來
1.透過ExplicitEuler更新衣服以及球體到deltaTime/2之後的position velocity

 
2.更新成未來的position velocity後就可以呼叫simulateOneStep取得未來的
速度以及加速度了
3.將未來的速度以及加速度儲存下來
4.呼叫過SimulateOneStep 須將position velocity acceleration恢復成原本狀態
5.根據MidPointEuler公式正確的更新未來的位置以及速度

```
### Integrator – ImplicitEuler 
```
我是先實作 MidPointEuler 
ImplicitEuler課程的部分再複習過一次之後發現
跟MidPointEuler是非常相似的 差別在我們不取中點而是直接取下一時間完成
下一個位置資訊 是由現在的位置 + deltaTime * 未來速度
下一個速度資訊 是由現在的位置 + deltaTime * 未來加速度
未來速度與一樣是透過ExplicitEuler計算出來
```

### Integrator – RungeKuttaFourth
 ```
根據公式把 k1 k2 k3 k4搞出來就結束了
主要從MidpointEuler有搞懂之後 這兩個積分器實作上都沒有太大問題
這個比較特殊的是在計算k2 k3 k4時都是從x0加上去的
也就是說每次呼叫完SimulateOneStep()之後 只要拿到f(….)裡面的內容
就要將origin的position velocity acceleration恢復再去計算後面的k
 
code部分的註解大概列出我實作的流程
0.儲存原本粒子狀態
1.K1可以直接用deltaTime * particle->velocity()算出來
算完之後將位置與速度透過k1來更新 ( 如同公式x0 + k1 /2 )

接續1.更新位置跟速度之後 就一樣可以透過simulateOneStep() 
取得這個時間點的速度以及加速度 這邊也就是獲得了f(x0+k1/2)

 
2.根據公式計算k2 因為我們已經算出了f(x0+k1/2)將他乘以deltaTime就可
3.為了繼續算之後的k3 k3部分的 f(x0+k2/2) 也是從x0的部分開始偏移
所以3.做的事情是將origin的資料恢復回去 計算才會正確
 
4.5.6 是在計算 k3 k4的流程 與上面計算k1 k2的味道一樣不再贅述
7.最後算完k4之後一樣需恢復到origin狀態
8.所有該求得的東西已經求取完畢 這邊才是積分器最主要的目的「更新下一個位置」「更新下一個速度」 套用公式就完成了
```

## Discussion
### String的連接
```
在實作之中需要我們連接三種不同類型的彈簧 (structural / bend / shear)
而且有特別提醒不要重複建立兩個粒子之間的彈簧
不過這邊就有好奇到 structural以及bend在本質上是很類似的彈簧 (方向)
為何需要多建立這部分的彈簧?

所以有嘗試註解掉bend類型的彈簧 發現模擬的結果是差不多的可以正常運作
差別在布料下落凹陷的幅度明顯增大 感覺更像材質比較軟的布料
 
實作中大概可以明白 彈簧的建立主要是為了考量該粒子的受力會受到周圍的哪些粒子的影響 
因為每個粒子少了隔一個粒子外的粒子的制約 讓布料之間沒辦法被更遠的粒子拉回來 才導致下陷幅度增加
所以透過觀察大概可以發現 我們可以透過彈簧的建立模式
來模擬球體落在不同材質布料的情況
至於要如何排列彈簧的連接方式來模擬不同材質布料 或許也是一項研究的方向
```

### difference between integrator 
這四種積分器撞起來其實大同小異 不過還是可以觀察出些微不同
```
ExplicitEuler: 撞的非常整齊 四個球相撞時都剛好是兩兩相切
後續的多次碰撞也都是可以漂亮的剛好切齊
```
```
ImlicitEuler: 在第二次第三次撞擊之後 球體會開始產生些微的偏移而不會像
ExplicitEuler都剛好四個互相切齊 研判是因為在計算時位置的偏移都是乘以未來的速度來估計出來 
而導致球體之間或球體布料之間的碰撞角度數量有些偏移
而且使用的速度是未來一個deltaTime的距離 偏移的情況較為嚴重一點
```
```
MidpointEuler: 這個的實作邏輯與ImplicitEuler類似 所以結果也差不多
在第二次第三次撞擊之後 球體會開始產生些微的偏移 
而其與Implicit不同之處在於它只會參考1/2 * deltaTime之後的速度來計算下一個位置與速度
所以觀察下來他偏移的情況明顯比Implicit好很多 雖然二三次後的相撞也非為同時互相切齊相撞 但也非常接近同時相撞了
```
```
Runge-Kutta 4th: 實作上最麻煩的一個 但是這樣的麻煩感覺也是有值得
在使用此積分器觀察碰撞時 也可以達成固定完美切齊的情形
主要應該是因為他中間取過多次距離外的速度來做彼此校準
前面取得較多 他後面取的就會幫忙修正回來 來讓精準度提高很多
```
``` 
照理來說 四球質量相同 都從方形布料上方同樣高度落下且彼此之間距離固定 不考慮其他摩擦力的情況 
正確的結果應該是四球穩定的固定彼此相切碰撞
所以從結果來看準確度排名應該是：
Runge-Kutta = ExplicitEuler > MidpointEuler > ImplicitEuler

不過按照我認知的感覺來說 感覺精確度的排行應該要是
Runge-Kutta > MidpointEuler > ImplicitEuler > ExplicitEuler才對
為什麼ExplictEuler反而會這麼靠前呢? 我可以猜到的理由是應該是湊巧而已?
或許在別的情況的模擬之下 Midpoint Implicit的通用性比較好 比較可以模擬出靠近正確物理結果的情況
而Explicit因為只用當下的速度來更新之後的位置 或許在某些情況可以表現得很完美 但放大來看 通用性或許就沒有來的這麼彈性

另外有一點很有趣的是 當我把deltaTime提高到20 Implicit的布料會爆掉
把deltaTime提高到30的時候Explicit Midpoint的布料都會直接爆掉 
但是Runge-Kutta卻可以正常運作
一直到將deltaTime提高到40時 Runge-Kutta才會一樣爆掉
也可以觀察出 Runge-Kutta在計算上因為是多點平均的結果 對於大偏移時間的耐受力肯定會比較好 所以這邊也猜測 
如果我們將Runge-Kutta的層數繼續往上加 只要我們硬體效能足夠 應該可以容忍到很大的deltaTime都不會使系統爆掉
```

### Effect of parameter
``` 
deltaTime: 隨著deltaTime的增加將會使偏移更加的嚴重
因為中間計算的次數大量的減少
```
```
springCoef: 彈簧的係數 觀察公式 F = -k *deltaX
彈簧的係數越大就代表這個彈簧越硬 => 硬彈簧不容易產生形變
也可以說當產生一樣的形變deltaX時 彈簧想要恢復成原本狀態的力量就更大
所以可以看到大彈性係數的布料凹陷情況非常少 布料的彈性也會增加
```
```
damperCoef : 阻力公式的部分類似彈簧 只是將形變的部分改成速度的變化量
所謂的阻力就是物體會偏好以原本的狀態運動 當速度今天要變快或是變慢時
會有一個力量希望讓物體保持原來的運動狀態
當damperCoef為0時 因為能量守恆的關係 當外力介入系統時 沒有地方讓系統消耗能量 將會導致系統一直運動下去而不會靜止
 
當damperCoef越大時 物體希望保持原來狀態的強度就會增加 所以球一落下後
能量馬上就被阻力吃光 導致球體布料不太會運動
damperCoef =1700 可以觀察到球下落一碰撞能量就小了很多
球體之間在碰撞2-3次後基本上就停下來了

當damperCoef越小時 物體希望保持原來狀態的強度就會減弱
所以只要一有外力介入系統讓系統有能量 系統這個能量就不容易被消耗
而讓球體不斷的運動而不會停下來
 ```
