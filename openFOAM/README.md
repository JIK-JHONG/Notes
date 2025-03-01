Image_Intensity_analysis
-
Analysis the image with RGB or Gray Intensity (C++ / Python)

| Program | Type | Description |
|-------|-------|-------|
| [Image_Intensity_analysis](https://github.com/JIK-JHONG/side_project/blob/main/Image_Intensity_analysis/Image_Intensity_analysis_cpp.cpp) | C++ / OpenCV + Gnuplot |Image processing |
| [Image_Intensity_analysis](https://github.com/JIK-JHONG/side_project/blob/main/Image_Intensity_analysis/Image_Intensity_analysis.py) | Python / OpenCV |Image processing |


| Parameter | type | Description |
|-------|-------|-------|
| input_image | Mat | image input |
| option | string | all / div / mix  |


| Original | Modified |
|-------|-------|
| ![Original](https://github.com/JIK-JHONG/side_project/blob/main/Image_Intensity_analysis/cat_img.jpeg) | ![Original](https://github.com/JIK-JHONG/side_project/blob/main/Image_Intensity_analysis/output_rgb_mix_py.png) |


openFOAM 安裝方式
-
**For Apple Silicon CPU**

透過GitHub 的來源，來產生 openFOAM.app (不使用官方Docker安裝法)
[openFOAM.app](https://github.com/gerlero/openfoam-app)


**修改檔案路徑：（當次有效）**

export FOAM_RUN=/Volumes/RamDisk/openFOAM

**拷貝範例至對應執行質料夾（需到該目錄，如：/Randisk/）**

cp -r $FOAM_TUTORIALS/heatTransfer/buoyantSimpleFoam/circuitBoardCooling .

語法為：（遞迴拷貝 cp -r）

cp -r $FOAM_TUTORIALS/<路徑> .


**處理 stl.**

surfaceFeatureExtract	截取特徵

snappyHexMesh			進行細化

subsetMesh -overwrite overFlowWall -patch walls

主要的功能是 提取 walls patch 上的網格，並將結果存入新的 mesh 區域 overFlowWall，同時覆蓋 (-overwrite) 現有網格資料。


指令解析
-
1.	subsetMesh：擷取部分網格（submesh）。
2.	-overwrite：覆蓋現有的 polyMesh（否則會輸出到 constant/polyMeshNew/）。
3.	overFlowWall：新建立的 mesh 區域名稱。
4.	-patch walls：從 walls 這個 patch 擷取對應的網格。


decomposePar			分配平行運算器		可以把任務分配至不同ＣＰＵ進行運算

mpiexec				執行平行運算
					mpiexec -n 9 interPhaseChangeFoam -parallel

					‧ 9							分配 9 個計算單元

					需要跟decomposeParDict 文件內容相同：
					
					numberOfSubdomains 9;  	//	N cores
					method          simple;  	//	平行運算切割方式
					coeffs
					{     	
						n           (3 3 1);	//	x * y * z = N cores;
					}


					‧ interPhaseChangeFoam		依據controllDict設定之求解器進行設定（需一樣）

reconstructPar			合併平行運算的數據

snappyHexMesh		額外處理外部 stl 檔案，進行Mesh

					snappyHexMesh -overwrite		
					會直接生成完整的 polyMesh，而不會顯示由「原始」>「細化」各階段的模型


blockMeshDict.m4		需要進行額外處理，輸入下列指令來轉換為「blockMeshDict」
					m4<system/blockMeshDict.m4>system/blockMeshDict

取得紀錄
					icoFoam > log &		演算處理器 > log &
					Cat log				觀看 log

後處理列表
					postProcess -list
					postProcess -funcs '(components(U) mag(U))'




雷諾數（Reynolds number）是流體的重力黏滯
Re = ρVL/μ = VL/ν 

V 	特徵速度		m/s
L	特徵長度		m
μ	流體動力黏度	Pa·s或N·s/m²
ν	流體運動黏度	m²/s
ρ	流體密度		kg/m³

┣ 湍流	Turbulence		Re > 4000
┣ 過度層	Transitional		2000 < Re < 4000				
┣ 流層	Laminar		Re < 2000



湍流（Turbulence） 
湍流是一種混亂、不可預測的流體運動狀態，具有以下特徵：  
1. 不規則性（Irregularity）：速度場變化劇烈，無法用簡單的數學方程式描述。  
2. 渦旋結構（Vortex Structures）：流體內部形成各種大小的渦流，並且這些渦旋會相互影響。  
3. 能量傳遞（Energy Cascade）：大尺度的渦旋將能量傳遞到小尺度的渦旋，最後因為黏性作用而轉換成熱能消散。  
4. 高動量與能量混合（Enhanced Mixing）：湍流能夠快速混合流體，影響熱傳遞與質量交換。  
5. 雷諾數高（High Reynolds Number）：湍流通常發生在雷諾數（Re）較高的情況下，例如 Re > 4000 的管流。

┗ 湍流數值模擬方法
	┣ 直接數值模擬（DNS - Direct Numerical Simulation）
		—>最精確，但計算量極大。
	┣ 大渦模擬（LES - Large Eddy Simulation）
		—>解析大渦旋，計算量中等。
	┗ 雷諾平均納維-斯托克斯方程（RANS - Reynolds-Averaged Navier-Stokes）
		—>最常見，適合工程應用。
		


步驟：
生成網格

blockMesh
注意，需要滿足右手定律（順序）

// Simple block

//     3-----2      y+|
//   7/|   6/|        |   
//   | 0---|-1        o-----
//   | /   | /       /     x+
//   4-----5      z+/  
//                      



blockMesh 主要結構體
┣ scale			1	單位換算，openFOAM 基礎單位為 m , 
					FreeCAD 為 mm >>> 假設換算就是 0.001 = 10^-3
┣ vertices			定義頂點位置（x,y,z）>>> 會對應到後面的計算單位立方體代碼 hex()
				(0, 0, 0)	// 	0
				(1, 0, 0)	//	1
				(1, 1, 0)	//	2
				(0, 1, 0)	//	3
┣ blocks			定義計算單位立方體，為8個點組成。（注意，這邊需要符合右手坐標系定律，
				也就是，(0,0,0)>(1,0,0)>(1,1,0)>(0,1,0); (0,0,1)>(1,0,1)>(1,1,1)>(0,1,1)）
				可以由多個hex組成。

				hex (0 3 2 1 4 7 6 5) (40 20 60) simpleGrading (1 1 1)
				hex = 定義基本計算單元（立方體座標點編號，參照vertices順序）
				(X_Block Y_Block Z_Block)
				simpleGrading	//	是否為漸變切細（1 1 1）表示個方向都是等比切細
				(
					1
					1
					1
				)
——	
漸變設定範例		
blocks
(
    hex (0 1 2 3 4 5 6 7) (30 20 10) simpleGrading (((0.5 0.5 10)(0.5 0.5 0.1)) 1 1)
);

網格(mesh)漸變設定
定義：
X：30 個網格，原本設定為 1 >>> (0.5 0.5 10)(0.5 0.5 0.1)
表示：

| ———0.5(15/30 meshes)——— | | ————0.5(15/30 meshes)———— |
|x1|                                 |    x2  | |    x3  |                                      |x4| 
最後一個網格的大小/第一個網格的大小(膨脹率)  =  x2 / x1 

(第一段占總長度比例	第一段網格總數的比例	最後一個網格的大小/第一個網格的大小(膨脹率，漸變率) )
(第二段占總長度比例	第二段網格總數的比例	最後一個網格的大小/第一個網格的大小(膨脹率，漸變率) )
…

——

┣ defaultPatch		如果未包含在邊界定義，都歸類在這邊
┗ boundary			邊界條件（一般常用）
	┣ inlet			進風（流體）口
	┣ outlet			出風（流體）口
	┣ top			頂部
	┣ bottom			底部
	┣ wall			牆
	┗ frontAndBack		側面


boundary
(
	inlet
	(
		type patch;
		faces
		(
			( 輸入座標點編號（參照hex定義）如：(0 1 2 3) )
		);
	);
);

一般常用邊界條件 type 類型 (boundary)
	‧	patch 			是最一般的邊界類型，允許靈活設定流體邊界條件。
	‧	wall 			適用於固體壁面。
	‧	symmetryPlane	適用於對稱問題。
	‧	empty 		適用於 2D 模擬。
	‧	wedge 		用於旋轉對稱問題。
	‧	cyclic 			用於週期性流場。
	‧	cyclicAMI 		用於非匹配的週期邊界 (如旋轉流場)。
	‧	processor 		用於 MPI 並行計算。



依照範例的方式，來執行不同的solver
如：

icoFOAM

顯示，模擬結果，
透過brew安裝套件：

brew install --cask paraview


paraview


一般計算檔案格式：

Project
┣ 0			初始條件：物理參數，如：壓力、速度等
	┣ alphat		m²/s		[0 2 -1 0 0 0 0]	熱擴散係數			熱對流問題
	┣ epsilon		m²/s³		[0 2 -3 0 0 0 0]	湍流耗散率			RANS k-ε 模型
	┣ k			m²/s²		[0 2 -2 0 0 0 0]	湍流動能			RANS 湍流模型
	┣ nut			m²/s		[0 2 -1 0 0 0 0]	黏度係數			RANS 湍流模型
	┣ p			Pa		[0 2 -2 0 0 0 0]	靜態運動壓力		低馬赫數不可壓縮流體
	┣ p_rgh		Pa		[0 2 -2 0 0 0 0]	相對運動壓力 p - ρgh	包含重力影響的不可壓縮流
	┣ T			K		[0 0 0 1 0 0 0]	溫度				熱傳或可壓縮流
	┣ U			m/s		[0 1 -1 0 0 0 0]	速度場			所有流體模擬
	┣ omega		1/s		[0 1 -1 0 0 0 0]	比耗散率（湍流尺度）	RANS k-ω 模型
	┣ phi			m³/s		[0 1 -1 0 0 0 0]	質量流率通量		常見於求解器
	┣ rho			kg/m³		[0 1 -1 0 0 0 0]	密度				可壓縮流
	┣ TKE		m²/s²		[0 1 -1 0 0 0 0]	亂流動能			LES 湍流
	┣ mut		kg/(m·s)	[1 -1 -1 0 0 0 0]	分子黏度			某些可壓縮流
	┗ …
┣ constant		
┗ system		邊界條件、幾何構圖等
	┣ blockMeshDict			網格設定，包含模型幾何、邊界條件
	┣ controlDict				控制參數：如運算量、切細等、輸出
	┣ fvSchemes
	┣ fvSolution
	┗ …


0 文件內

參數說明：
dimension 物理量 [A B C D E F G]
[重量 長度 時間 溫度 物質的量 電流 照度]
A	重量		kg
B	長度		m
C	時間		s
D	溫度		K
E	物質的量	mol
F	電流		A
G	照度		cd

如：速度 U  = m/s >>> B/C >>> [0 1 -1 0 0 0 0]

=========
速度		U	m/s				m‧s^-1				[0 1 -1 0 0 0 0]
壓力		P	pa				kg‧m‧s^-2			[1 -1 -2 0 0 0 0]
溫度		T	K				K					[0 0 0 1 0 0 0]
導熱率	ld	W/m/k			Kg‧m‧s^-3‧K^-1		[1 1 -3 -1 0 0 0]
擴散係數	D	m/s²				m‧s^-2				[0 1 -2 0 0 0 0]

能量源	S	W/m³			kg‧m^-1‧s^-3			[1 -1 -3 0 0 0 0]
密度		ρ	kg/m³			kg‧m^-3				[1 -3 0 0 0 0 0]
比熱容	Cp	J/(kg‧K)			m^2‧s^-2‧K^-1		[0 2 -2 0 -1 0 0]


運動壓力	P/ρ	Pa/kg‧m^3		m^2‧s^-2				[0 2 -2 0 0 0 0]

注意：當0文件內，只有給 p & U 則通常 P 是	運動壓力	[0 2 -2 0 0 0 0]


BoundaryField	邊界條件場

1. 固定值類型 (fixedValue 類)
邊界條件					說明
fixedValue				設定固定的數值，例如速度、壓力或溫度
uniformFixedValue			與 fixedValue 類似，但允許設定均勻分布的值
directionMixed			允許不同方向上有不同的固定值
codedFixedValue			允許使用程式碼（C++）來動態設定固定值

2. 流體動力學相關 (velocity, pressure)
邊界條件					說明
noSlip					無滑移邊界條件，速度固定為零（視為固體表面）
slip						滑移邊界，垂直於邊界的速度分量為零，但切向速度可變
zeroGradient				設定梯度為零，通常用於壓力出口，表示無壓力差
fixedFluxPressure			使壓力符合流動條件，自動調整以維持質量流率
totalPressure				設定總壓力，常用於入口或邊界處
inletOutlet				允許流體進出，通常用於開放邊界
pressureInletOutletVelocity	允許壓力驅動的流動，出口速度由壓力梯度決定
prghPressure		      主要用於出口 (outlet) 或自由液面 (free surface) 邊界，適用於：
					‧	VOF (自由液面) 模型 (interFoam)
					‧	可壓縮或浮力影響的流動 (buoyantPimpleFoam, buoyantSimpleFoam)
					‧	任何需要考慮重力影響的流動場
	

3. 熱傳導與溫度 (thermal, heat transfer)
邊界條件					說明
fixedTemperature			固定溫度邊界
zeroGradient				無溫度梯度，表示熱流自由通過
externalWallHeatFluxTemperature	設定外部熱通量，如對流或輻射熱流
compressible::turbulentTemperatureCoupledBaffleMixed						用於可壓縮流體的熱傳導
totalTemperature			設定總溫度（焓、內能）

4. 壓縮性流體 (compressible flow)
邊界條件					說明
totalPressure				設定總壓力，適用於可壓縮流體的入口
waveTransmissive			設定波動穿透邊界，允許壓縮波自由傳播
supersonicOutlet			超音速出口邊界條件
inletOutletTotalPressure	可壓縮流體的入口與出口壓力邊界條件

5. 網格與數值計算 (mesh motion, numerical schemes)
邊界條件					說明
empty					空邊界，適用於 2D 模擬（忽略該方向上的運算）
symmetryPlane			對稱平面條件
cyclic					週期性邊界條件，常用於流體力學或週期性結構
wedge					用於軸對稱問題，例如噴嘴或渦輪機葉片

6. 其他
邊界條件					說明
fixedValue				固定數值，可以取代fixedFluxPressure, fixedFluxPressure, fixedTemperature等
calculated				允許 OpenFOAM 根據內部計算來決定邊界值，通常作為其他條件的輔助設定

	‧	calculated 本身不會設定具體數值，而是讓 OpenFOAM 根據內部運算決定它的值。
	‧	常見於 zeroGradient、fixedFluxPressure 等邊界條件的內部計算。


	‧	流體進口 (inlet)：fixedValue（速度）、totalPressure（壓力）
	‧	流體出口 (outlet)：zeroGradient（壓力）、inletOutlet（速度）
	‧	固體邊界 (wall)：noSlip（流體）、fixedValue（溫度）
	‧	對稱邊界 (symmetry)：symmetryPlane
	‧	2D 模擬 (empty)：用於擠壓式 2D 模擬

打包設定，”(A|B|C|....)”
“(wall|top|bottom)”
(
	type noSlip;
)

k 文件
┗ k = 3/2 * ( V’ ) = 3/2 * ( 5% * v )^2
					   平均速度 5%
	對應初始條件 k 

alphat 文件
┗ Prt	0.75 ~ 0.95

epsilon 文件	湍流耗散率
湍流耗散率 (Turbulent Dissipation Rate) 	m^2/s^3
┗ epsilon = ( (C/v) ^ (3/4) * k ^ (3/2))/L
C	常數，通常與經驗模型參數有關，通常為 0.09				無單位	
v	運動黏度 (kinematic viscosity)						m^2/s
	v = u/p
	\mu：u 動態黏度 (Dynamic Viscosity, kg/(m·s))
	\rho：p 流體密度 (Density, kg/m^3)

K	湍流動能 (Turbulent Kinetic Energy, TKE)				m^2/s^2
L	湍流積分尺度 (Integral Length Scale)，代表湍流渦流的平均大小）m	
	，L = 2% ~ 10% * D (實際長度)
特徵長度尺度 (Length scale)，通常為湍流積分尺度 (Integral length scale)



=========
U 文件
internalField	內場		uniform (0 0 0) ：初始內部速度為0
如：V = Uxi + Uyj + Uzk 

BoundaryField	邊界條件場	注意，要與mesh定義相同。

type		fixedValue 	固定值
		noSlip		固定不動（視為鋼體）
		empty		空（對應幾何條件）
		zeorGradient	沒有壓力差
		empty		空（對應幾何條件）

value 	uniform		(1 0 0 )		表示	Ux * 1 + Uy * 0 + Uz * 0 = Ux 只有Ｘ方向 


p 文件

dimension 	物理量 	[0 2 -2 0 0 0 0]		如沒有設定密度，通常為「運動壓力，即 P/ρ」

internalField	內場		uniform 0 			初始內部壓力為與大氣壓力差為 0 ，表示與大氣壓力相同

BoundaryField	邊界條件場					注意，要與mesh定義相同。

type		fixedValue 	固定值
		noSlip		固定不動（視為鋼體）
		empty		空（對應幾何條件）
		zeorGradient	沒有壓力差
		empty		空（對應幾何條件）


T	文件

dimensions      [0 0 0 1 0 0 0];

internalField   uniform 100;


boundaryField
{
    fixedTemp
    {
        type            fixedGradient;  		// 固定熱流密度
        gradient        uniform 40;        // 單位: W/m²，這是熱流密度
	//	如果為一般溫度
	//	type            fixedValue;
	//	value           uniform 100; // Temp in Kelvin
    }

    insulated
    {
        type            zeroGradient;
        //type            fixedValue;
        //value           uniform 20; // Temp in Kelvin
    }

    frontBackTopBottom
    {
        type            empty;
    }
}



fixedGradient 物理上定義：

q = - k dT/dx  		q = 熱通量 (heat flux)  W/m^2
					K = 導熱率 w/mk
					dT/dx = 溫度梯度（temperature gradient），單位 K/m，
						     表示溫度沿著法向方向 ( n ) 的變化率

type fixedGradient;
gradient 10000;

這表示該邊界的溫度梯度固定為 10,000 K/m，也就是每公尺的溫度變化量為 10,000°C。

根據傅立葉定律：

q = -k \times (10000)

如果我們的材料熱導率  k  為 1 W/(m·K)（假設材料是一般的熱傳導介質，如某些金屬），那麼熱通量就是：

q = - (1) \times (10000) = -10000 \text{ W/m}^2

這表示該邊界每平方公尺有 10,000W 的熱流進入或流出。
	‧	如果 gradient 為正數，則溫度隨著法向方向增加，表示該表面是熱源（有能量輸入）。
	‧	如果 gradient 為負數，則溫度隨著法向方向降低，表示該表面是散熱區域（有能量輸出）。

什麼時候使用 fixedGradient？

你會在以下情況下使用：
	1.	模擬固定熱流輸入的表面
	‧	例如，一個發熱元件持續加熱一個物體
	2.	模擬固定熱流散熱的表面
	‧	例如，一個強制冷卻的金屬板
	3.	當你已經知道邊界的熱通量，而不是溫度時
	‧	例如，某個區域的熱流是已知的，但溫度是未知的

相對地，如果你只知道某個表面溫度，而不是熱流，那應該使用 fixedValue 邊界條件來設定一個固定的溫度值。
					

如果你想精確控制熱輸入功率（如 5W），
可以使用 externalWallHeatFluxTemperature 來直接設定總輸入熱功率，而不只是溫度梯度。






blockMeshDict 文件
blocks
(
    hex (0 1 2 3 4 5 6 7) (30 20 10) simpleGrading (((0.5 0.5 10)(0.5 0.5 0.1)) 1 1)
);

網格(mesh)漸變設定
定義：
X：30 個網格，原本設定為 1 >>> ( (0.5 0.5 10)(0.5 0.5 0.1) )
表示：

| ———0.5(15/30 meshes)——— | | ————0.5(15/30 meshes)———— |
|x1|                                 |    x2  | |    x3  |                                      |x4| 
最後一個網格的大小/第一個網格的大小(膨脹率)  =  x2 / x1 

(第一段占總長度比例	第一段網格總數的比例	最後一個網格的大小/第一個網格的大小(膨脹率，漸變率) )
(第二段占總長度比例	第二段網格總數的比例	最後一個網格的大小/第一個網格的大小(膨脹率，漸變率) )
…

網格拉伸
假設只有一段，可以簡化成「只設定最後一個數值，也就是漸變率」

hex (0 1 2 3 4 5 6 7) (30 20 10) simpleGrading ( 10 1 1 )




客製化模型資料結構：（針對熱傳）
heatSimulation/
├── 0/                   # 初始條件
│   ├── T               # 溫度場初始條件
│   ├── U               # 速度場（可忽略，僅熱傳模擬不需要）
│   ├── alphaEff     # 熱擴散率（若有變化）
│   ├── k               # 熱導率
│   ├── boundaryField   # 邊界條件
├── constant/           # 物理性質 & 幾何網格
│   ├── transportProperties # 材料的熱導率、熱擴散率
│   ├── thermophysicalProperties # 熱物理性質
│   ├── polyMesh/       # 網格資料
│       ├── blockMeshDict （如果用 blockMesh 建立）
│       ├── boundary    （如果用 snappyHexMesh）
├── system/             # 求解器控制
│   ├── controlDict     # 控制計算的步驟、時間
│   ├── fvSchemes       # 數值計算方法
│   ├── fvSolution      # 求解方法
│   ├── decomposeParDict # 如果使用並行計算
│   ├── topoSetDict     # 網格選取（可能需要）
│   ├── createPatchDict # 如果需要新增 boundary patch


SnappyHexMesh 文件說明

castellatedMesh on;	網格細分
snap            on;		網格貼合
addLayers       off;		生成邊界層


explicitFeatureSnap    false;	通常外部檔案為 true
implicitFeatureSnap    true;	通常內部檔案為 true





客製化模型流程：
BlcokMesh —> SurfaceFeatureExtract —> SnappyHexMesh

二進位stl 轉 ascii (純數) stl
surfaceConvert constant/triSurface/carb.stl constant/triSurface/carb_ascii.stl

從FreeCAD匯入檔案
1.先轉到 mesh 頁面，並點擊「輸出成網格」，需要存檔成 ascii 之 stl格式
2.之後把檔案放到對應資料夾（triSurface），並輸入： 
**system 中需要包含：snappyHexMeshDict , surfaceFeatureExtractDict


castellatedMesh on;	//	→ 產生階梯狀的初始網格
snap            on;		//	→ 讓網格貼合幾何表面
addLayers       off;		//	→ 不加邊界層

1.	castellatedMesh on;
	‧	這個參數決定是否要執行「階梯式網格」（Castellated Mesh）生成階段。
	‧	on：會根據背景網格（通常是 blockMesh 生成的六面體網格）切割出符合幾何形狀的粗略網格，並移除不需要的內部單元。
	‧	off：不進行此步驟。
	2.	snap on;
	‧	這個參數決定是否執行「貼合網格」（Snapping）階段。
	‧	on：會讓 Castellated Mesh 進一步貼合幾何表面，使網格更接近真實的幾何形狀。這個步驟通常使用 surfaceFeatureExtract 來輔助細部特徵的捕捉。
	‧	off：不進行貼合，會保持階梯式的粗略網格。
	3.	addLayers off;
	‧	這個參數決定是否要在幾何邊界上添加「層狀網格」（Boundary Layers），這對於模擬流體邊界層行為非常重要。
	‧	on：會在物體表面附近加上額外的薄層網格，以更精細地捕捉邊界層的影響。
	‧	off：不添加層狀網格，適合不關注邊界層效應的模擬（例如初步測試）。


geometry
{
    car
    {
        type triSurfaceMesh;
        file "car_ascii.stl";
    }

	讀取外部的結構檔案，
	type triSurfaceMesh;
	file “檔名”

    refinementBox
    {
        type searchableBox;
        min  (  -41   -21   -16);
        max  (41 21  16);
    }

	refinementBox（細化區域框），使該區域內的網格更加精細。，
	type triSurfaceMesh;
	min  (  -41   -21   -16);	定義長方體的最小坐標（左下角或負方向的邊界）。
        	max  (41 21  16);		定義長方體的最大坐標（右上角或正方向的邊界）。

}




castellatedMeshControls
{
    features
    (
      { file  "car_ascii.eMesh"; level 1; }
    );
	提取特徵(提取幾何模型的銳邊（edges），確保細節不會被過度平滑處理。)，
	需要先執行	surfaceFeatureExtractDict
	才會產生 car_ascii.eMesh
	level 1;：設定細化等級，數值越大表示越精細。
	‧	Level 1 細化：每個單元被切割成 2×2×2 = 8 個小單元
	‧	Level 2 細化：每個小單元再被切割 2×2×2 = 8 倍，即總共 64 倍
	‧	Level N 細化：每個單元會被細分為  8^N  個小單元


    refinementSurfaces
    {
        car
        {
            level (1 2);
            patchInfo { type wall; }
        }
    }
	‧	作用：控制特定幾何表面（如 car）的細化程度。
	‧	level (1 2);：
	‧	(最小細化等級 最大小化等級)，例如：
	‧	1 → 最低細化等級
	‧	2 → 最高細化等級
	‧	這表示該表面上的網格將根據需要自動細化到 1 或 2 的等級。
	‧	patchInfo { type wall; }：
	‧	指定此表面在 OpenFOAM 中的邊界條件類型。
	‧	wall 表示此面將作為固體壁面。


    refinementRegions
    {
        refinementBox
        {
            mode inside;
            levels ((1 1));
        }
    }
	‧	作用：在 refinementBox 內細化網格，提高該區域的計算精度。
	‧	mode inside;：
	‧	只在 refinementBox 內細化，不影響外部區域。
	‧	levels ((1 1));：
	‧	(最小等級 最大等級)，此處 1 1 表示在該區域內，細化至等級 1。


    locationInMesh (43 23 18);
	‧	作用：確保 snappyHexMesh 內外區分 正確。
	‧	這個點 (43, 23, 18) 必須在計算域內部（但須在物件外（即：非car區域）），否則網格可能會出錯。

}

snapControls
{
    explicitFeatureSnap    true;
    implicitFeatureSnap    false;
}
	‧	explicitFeatureSnap true;
	‧	開啟 顯式特徵貼合，讓網格貼合 car_ascii.eMesh 中的銳邊。
	‧	implicitFeatureSnap false;
	‧	不使用 隱式特徵貼合（這通常用於沒有 .eMesh 的情況）。

addLayersControls
{
    layers
    {
        face
        {
            nSurfaceLayers 2;
        }
	‧	作用：在 face 邊界（如 car）附近添加 2 層網格，以更精確地模擬邊界層效應。

    }

    relativeSizes       true;
	‧	true：邊界層厚度相對於附近的網格尺寸。
	‧	false：邊界層厚度為絕對值。


    expansionRatio      1.2;
	‧	這個值控制每層之間的厚度增加比率（如 1.2 表示每層厚度是前一層的 1.2 倍）。


    finalLayerThickness 0.5;
	‧	最外層的厚度比例，0.5 表示相對於總邊界層厚度的 50%。

    minThickness        1e-3;
	‧	作用：控制網格品質（如扭曲角度、邊長比等）。
	‧	這裡是空的，表示採用 OpenFOAM 預設的品質控制。

}

meshQualityControls
{}

writeFlags
(
    noRefinement
    // scalarLevels
    // layerSets
    // layerFields
);
	‧	noRefinement：不輸出細化層資訊。
	‧	scalarLevels（註解掉）→ 可選：輸出標量細化層資訊。
	‧	layerSets（註解掉）→ 可選：輸出邊界層集合資訊。
	‧	layerFields（註解掉）→ 可選：輸出邊界層場數據。

mergeTolerance 1e-5;
	‧	作用：定義節點合併時的距離容差。
	‧	若兩個節點距離小於 1e-5，則合併為一個節點，以確保網格連續性。




fvSchemes	求解（方案）器


梯度			grad()
旋度			div()
二次梯度		laplacian()

Header 

FoamFile
{
    version     2.0;
    format      ascii;
    class       dictionary;
    location    "system";
    object      fvSchemes;
}

ddtSchemes
{
    default         steadyState;
}
ddtSchemes 用於設置時間微分（對時間的導數）方案。default 表示預設情況下的時間積分方法：
	‧	steadyState: 表示穩態求解，意味著該問題不涉及時間依賴性，因此不需要時間微分。
		這樣，模擬不會隨時間發展（穩態模擬）。
如果這是瞬態（time-dependent）模擬，則通常會選擇其他時間積分方法，如 Euler。

	•	steadyState		穩態問題，不需要時間積分。
	•	Euler			簡單的顯式方法，適用於小時間步。
	•	CrankNicolson	隱式方法，適用於更穩定的瞬態問題。
	⁃	CrankNicholson 0.9; 其中 0.9 代表 90% 隱式、10% 顯式的混合方法。
	•	BDF1			適用於長時間模擬，尤其是穩定性較好的情況。
	•	RungeKutta		高精度的顯式方法，適用於精度要求高的瞬態問題。


gradSchemes
{
    default         Gauss linear;
    grad(T)         Gauss linear;
}
gradSchemes 用於定義梯度（grad）的計算方法。梯度是對場（如速度、壓力、溫度等）的空間導數。這裡的設置有：
	‧	default: 預設情況下使用 Gauss linear 方法來計算梯度。
	‧	grad(T): 針對溫度場（T）計算梯度，也使用 Gauss linear 方法。
Gauss linear 表示使用線性插值方法來計算梯度。

其他方式有，
Gauss linearUpwindGrad;（上風法）	這個方法是對 Gauss linear 的一種改進，適用於處理 上風法（upwind）方案。
								在這種方法中，會考慮相鄰網格點的影響來推導梯度。
								適用於解決 對流 dominate 的問題（如流體動力學中的速度場或熱場）。

Gauss cubic;						這種方法使用 三次插值 方法來計算梯度。這比 linear 方法提供更高的精度，
								尤其在場值變化較為平滑的情況下，但在不均勻網格或強變化區域中可能會不太穩定。

Gauss linearCorrected;				這個方法是在 Gauss linear 基礎上進行修正，旨在提高數值穩定性，
								尤其是在 剪切流 或是 不規則網格 上。它對於避免數值震盪和提高精度非常有效。

cellLimitedGauss;					這是一種針對 有限體積方法（Finite Volume Method，FVM）設計的計算方法，
								對於在不均勻網格上處理梯度的情況有較好的表現。
								這方法在計算過程中會對梯度進行局部限制，避免某些區域的梯度過大。

leastSquares;						這個方法使用 最小二乘法（least squares）來計算梯度。
								這是一種通過最小化誤差來估算梯度的方法，適用於不規則的網格和需要較高精度的問題。

smooth;							這種方法會對梯度進行平滑處理，適用於需要去除數值噪聲的情況。
								在某些情況下，這種方法可以減少不穩定性，但可能會犧牲精度。

	‧	Gauss linear：最常見的方法，適用於大多數情況，尤其是對於流場或溫度場的計算。
	‧	Gauss cubic：當需要更高精度時可以選擇，但對於不規則網格，可能會導致數值不穩定。
	‧	Gauss linearCorrected：對於需要穩定性（如強剪切流）的問題非常有用。
	‧	leastSquares：適合對不規則網格進行計算，精度較高，計算複雜度也較大。
	‧	cellLimitedGauss：用於有限體積方法，對於不規則網格具有較好的表現。



divSchemes
{
    default         none;
}
divSchemes 用來設置散度（div）的離散化方法。散度是向量場的數學操作，表示場的發散程度：
	‧	default: 預設情況下不使用任何散度離散化（none）。
這種設置適用於穩態或不需要散度計算的情況。在需要數值求解的情況下，通常會選擇例如 Gauss upwind 等方法。


laplacianSchemes
{
    default         none;
    laplacian(DT,T) Gauss linear corrected;
}
laplacianSchemes 用來設置拉普拉斯算子（laplacian）的離散化方法。拉普拉斯算子通常出現在熱傳遞或流體力學的問題中：
	‧	default: 預設情況下不使用任何拉普拉斯離散化（none）。
	‧	laplacian(DT,T): 針對溫度場 T 和時間步長 DT 設置離散化方法。
		這裡使用 Gauss linear corrected，表示使用線性插值方法並進行校正。

Gauss linear corrected 方法適用於處理帶有修正項的問題，像是穩定性較差的數值方法。

interpolationSchemes
{
    default         linear;
}
interpolationSchemes 用來設置場值插值方法。插值方法用來在網格點之間估算值：
	‧	default: 預設情況下使用 linear 插值方法，這是一個線性插值方法。

插值方法常用於計算面積、體積等屬性或在數值求解過程中將場值從一個網格位置插值到另一個位置。


snGradSchemes
{
    default         corrected;
}

snGradSchemes 用來設置表面梯度（snGrad）的計算方法。表面梯度是指在邊界面上的場變化率：
	‧	default: 預設情況下使用 corrected 方法，這是一種修正的梯度計算方法，
		用來提高數值穩定性，特別是對於剪切流動等問題。






transformPoints -scale (0.001 0.001 0.001)

來源檔案位置：fan_unit_meshed
對應新檔案位置：fan_unit_output.stl

(將把座標重新分配，到原點(0,0,0)，並用對應比例 0.001進行換算，一般來說 FreeCAD預設為 mm ， openFOAM預設為 m ，所以為，10^-3 =0.001 )
來源檔案位置 對應新檔案位置 -clean -scale 0.001

fan_unit_meshed.stl fan_unit_output.stl -clean -scale 0.001


surfaceTransformPoints bullet.stl output.stl -scale 0.0005

surfaceTransformPoints constant/triSurface/heat_sink.stl constant/triSurface/heat_sink_md.stl -scale 0.001




transformPoints -rotate "90 z" output.stl fan_unit_output.stl

surfaceConvert input_binary.stl output_ascii.stl

平移
surfaceTransformPoints source target -translate '(0 0 -20)'
surfaceTransformPoints car.stl car_transformed.stl -translate '(0 0 -20)'


檢查網格匹配（確認是否可以使用該mesh）
checkMesh


1. 單一相變熱傳導
這類求解器專注於熱擴散方程，適用於純熱傳導問題，例如固體內部的熱擴散過程。

laplacianFoam		
	暫態（Transient），傅立葉熱傳導方程，僅考慮熱擴散，適用於不可壓縮流體或固體	- 金屬內部熱擴散- 固體材料內的溫度分布演變
scalarTransportFoam	
	穩態（Steady-state），被動標量輸運（例如溫度），假設已知速度場	- 溫度場輸運分析（但不考慮對流影響）- 監測特定區域的溫度變化

2. 對流換熱（自然與強制對流）
這類求解器考慮了流場對溫度場的影響，通常涉及自然對流（Boussinesq 假設）或強制對流（壓縮性流動）。

buoyantBoussinesqSimpleFoam	
	穩態（Steady-state），自然對流，不可壓縮，使用 Boussinesq 假設	- 室內空氣對流（冷熱源影響）- 電子設備散熱分析
buoyantBoussinesqPimpleFoam	
	暫態（Transient），自然對流，不可壓縮，使用 Boussinesq 假設	- 時變熱源影響的流場（如加熱元件啟動過程）
buoyantSimpleFoam	
	穩態（Steady-state），自然對流，可壓縮（低速），可考慮輻射影響	- 高溫環境內的對流分析（如爐內流場）- 建築 HVAC 系統分析
buoyantPimpleFoam	
	暫態（Transient），自然對流，可壓縮（低速），可考慮輻射影響	- 高溫設備的瞬態熱影響（如焊接過程）


3. 壓縮性流體熱傳
這類求解器適用於高溫、高速或可壓縮性流動的熱傳問題，通常與動力設備、燃燒、噴射等相關。

chtMultiRegionFoam	
	暫態（Transient），可壓縮，共軛熱傳	
	- 電子散熱（如 CPU、散熱片）- 熱交換器分析（如冷卻系統）
chtMultiRegionSimpleFoam
	穩態（Steady-state），可壓縮，共軛熱傳	
	- 與 chtMultiRegionFoam 類似，但適用於穩態問題

4. 固-流耦合熱傳（共軛熱傳 - CHT, Conjugate Heat Transfer）
這類求解器考慮固體與流體之間的熱交換，適用於電子散熱、熱交換器、金屬與流體的熱傳導問題。

chtMultiRegionFoam	
	暫態（Transient），可壓縮，共軛熱傳	
	- 電子散熱（如 CPU、散熱片）- 熱交換器分析（如冷卻系統）
chtMultiRegionSimpleFoam	
	穩態（Steady-state），可壓縮，共軛熱傳	
	- 與 chtMultiRegionFoam 類似，但適用於穩態問題

5. 熱物性演化
這類求解器關注熱物性的變化，但不影響速度場。
thermoFoam	
	暫態（Transient），在固定速度場條件下演化熱物性	
	- 熱擴散過程中的物性變化

6. 額外的熱傳求解器
這些求解器專門處理熱傳、熱輻射或能量守恆問題。

heatTransferFoam	
	暫態（Transient），熱傳導與能量守恆方程，適用於固體或流體	- 熱交換系統- 非穩態熱傳擴散
potentialFoam	
	穩態（Steady-state），潛勢流求解器，雖非專門的熱求解器，但可用於初始化熱流場	- 初始場計算（熱流模擬前的速度場初始化）


7. 考慮輻射傳熱的求解器
這些求解器除了熱對流與熱傳導外，也考慮輻射影響，適用於高溫環境。

buoyantRadiationFoam	
	暫態（Transient），考慮自然對流與輻射熱交換	
	- 高溫環境的熱場分析（如熱爐內部溫度場）
rhoCentralFoam	
	暫態（Transient），適用於可壓縮高溫氣體流動，包含熱傳導與輻射	- 高溫噴射流（如火箭噴嘴內部流場）

8. 更高階的共軛熱傳求解器
這些求解器專門針對固-流熱耦合問題，比 chtMultiRegionFoam 更進階。

chtMultiRegionTwoPhaseFoam	
	暫態（Transient），共軛熱傳導，適用於兩相流體（如冷卻劑與氣體交互換熱）	- 冷卻系統設計（如冷卻水與金屬管壁的換熱分析）
compressibleMultiphaseInterFoam	
	暫態（Transient），可壓縮的多相流求解器，考慮氣-液相變（如蒸發、冷凝）	- 冷卻液蒸發過程（如電子冷卻應用）

9. 與能量方程相關的求解器
這些求解器雖然主要用於流體模擬，但也可處理熱傳問題。

rhoReactingFoam	
	暫態（Transient），可壓縮、包含能量方程，適用於燃燒反應與熱傳遞	- 燃燒室溫度場計算
reactingFoam	
	暫態（Transient），適用於低速可壓縮反應流，包含能量守恆方程	- 化學反應熱傳問題

10. 熱機械分析（耦合固體應力與熱傳）
這些求解器考慮了固體熱傳與應力場的影響，適用於溫度變化導致熱應力的應用。

solidDisplacementFoam	
	暫態（Transient），固體應力與熱變形分析，適合與熱傳導耦合	- 溫度變化導致的應力場變化（如熱膨脹）
thermoMechanicalFoam	
	暫態（Transient），考慮熱傳與應力場的求解器	- 高溫結構件變形分析



icoFoam	
	暫態（Transient），不可壓縮	❌ 不考慮熱傳	低雷諾數的層流流動
simpleFoam	
	穩態（Steady-state），不可壓縮	❌ 不考慮熱傳	紊流模擬、風洞、管道流動

⚠ 如果需要模擬熱傳導或對流，請考慮：
	‧	buoyantSimpleFoam（穩態，自然對流，考慮熱浮力）
	‧	chtMultiRegionFoam（暫態，共軛熱傳，適用於固體與流體交互影響）




OpenFOAM solvers for heat transfer analysis

laplacianFoam:
Transient, incompressible, thermal diffusion according to Fourier’s law

scalarTransportFoam:
Steady-state, incompressible, laminar, passive scalar e.g. temperature for a given velocity field

buoyantBoussinesqSimpleFoam:
Steady-state, thermal, natural convection, incompressible,Boussinesq’s approximation

buoyantBoussinesqPimpleFoam:
Transient, thermal, natural convection, incompressible, Boussinesq’s
approximation

buoyantSimpleFoam:
Steady-state, natural convection, compressible (sub-sonic), including radiation

buoyantPimpleFoam:
transient, natural convection, compressible(sub-sonic), including radiation

rhoSimpleFoam:
Steady-state, thermal, compressible(sub-sonic)

rhoSimplecFoam:
Steady-state, thermal, compressible(sub-sonic) -Pressure under relaxiation =1

rhoPimpleFoam:
Transient, thermal, compressible(sub-sonic)

chtMultiRegionFoam:
Transient, compressible, conjugate heat transfer between solid and fluid

chtMultiRegionSimpleFoam:
Steady-state, compressible, conjugate heat transfer between solid and fluid

thermoFoam:
Transient, evolves the thermophysical properties for a frozen velocity field



選擇求解器的關鍵考量
	1.	是否考慮對流影響
	‧	只關心熱傳導 → laplacianFoam
	‧	考慮溫度場輸運，但流場已知 → scalarTransportFoam
	‧	需要對流影響（自然對流） → buoyantBoussinesqSimpleFoam（穩態）或 buoyantBoussinesqPimpleFoam（暫態）
	‧	需要對流影響（可壓縮） → buoyantSimpleFoam 或 buoyantPimpleFoam
	2.	是否為暫態或穩態
	‧	穩態分析 → SimpleFoam 家族（Simple = Steady）
	‧	暫態分析 → PimpleFoam 家族（Pimple = Transient）
	3.	是否為可壓縮流動
	‧	不可壓縮：buoyantBoussinesqSimpleFoam, buoyantBoussinesqPimpleFoam
	‧	可壓縮（低速）：buoyantSimpleFoam, buoyantPimpleFoam, rhoSimpleFoam, rhoPimpleFoam
	4.	是否涉及固-流共軛熱傳
	‧	需要考慮固-流界面傳熱 → chtMultiRegionFoam（暫態）或 chtMultiRegionSimpleFoam（穩態）

選擇求解器的關鍵考量
	1.	是否考慮對流影響
	‧	只關心熱傳導 → laplacianFoam
	‧	考慮溫度場輸運，但流場已知 → scalarTransportFoam
	‧	需要對流影響（自然對流） → buoyantBoussinesqSimpleFoam（穩態）或 buoyantBoussinesqPimpleFoam（暫態）
	‧	需要對流影響（可壓縮） → buoyantSimpleFoam 或 buoyantPimpleFoam
	2.	是否為暫態或穩態
	‧	穩態分析 → SimpleFoam 家族（Simple = Steady）
	‧	暫態分析 → PimpleFoam 家族（Pimple = Transient）
	3.	是否為可壓縮流動
	‧	不可壓縮：buoyantBoussinesqSimpleFoam, buoyantBoussinesqPimpleFoam
	‧	可壓縮（低速）：buoyantSimpleFoam, buoyantPimpleFoam, rhoSimpleFoam, rhoPimpleFoam
	4.	是否涉及固-流共軛熱傳
	‧	需要考慮固-流界面傳熱 → chtMultiRegionFoam（暫態）或 chtMultiRegionSimpleFoam（穩態）




0. 先下載 docker for Apple Silicon
https://www.docker.com/get-started/


1. 更新Rosetta2
softwareupdate --install-rosetta

2.






