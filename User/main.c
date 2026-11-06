#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "HC05.h"
#include "PID.h"
#include <math.h>

/* ==================== 全局变量 ==================== */

/* MPU6050 的原始数据：AX/AY/AZ 是三轴加速度，GX/GY/GZ 是三轴角速度 */
int16_t AX, AY, AZ, GX, GY, GZ;

/* 定时器超时标记：中断里清完标志位后标志位又被置 1，说明本次中断执行超过了 1ms */
uint8_t TimerErrorFlag;

/* 中断退出瞬间的定时器计数值，可用来估算中断函数执行了多久 */
uint16_t TimerCount;

/* 加速度计解算出的角度 */
float AngleAcc;

/* 陀螺仪积分得到的角度，互补滤波后它与 Angle 基本重合 */
float AngleGyro;

/* 互补滤波结果：既没有加速度计的抖动，也没有陀螺仪的漂移，这是真正拿去做控制的角度 */
float Angle;

/* KeyNum 保存键码，RunFlag 为 1 表示平衡程序正在运行 */
uint8_t KeyNum, RunFlag;

/* 左右轮的 PWM 值，以及由它们合成/分解出的平均 PWM 与差分 PWM */
int16_t LeftPWM, RightPWM;
int16_t AvePWM, DifPWM;

/* 左右轮转速（电机输出轴，单位转/秒），以及平均速度、差分速度 */
float LeftSpeed, RightSpeed;
float AveSpeed, DifSpeed;

/* ==================== PID 参数 ==================== */

/* 角度环：最内环，10ms 一次，负责让车身保持直立
   它的输出直接换算成 PWM，所以这里的系数最敏感，改动要小步走 */
PID_t AnglePID = {
	.Kp = 2.4,						// 比例项：角度偏离多少就给多少力，决定"扶正"的劲道
	.Ki = 0,						// 积分项：角度环一般不用积分，避免缓慢积累导致车身漂移
	.Kd = 0.6,						// 微分项：这里用"微分先行"，相当于给车身加阻尼，压住来回摆动

	.OutMax = 100,					// 输出上限，调试期限制到 100，即使参数写错车也不会失控到飞起来
	.OutMin = -100,					// 输出下限

	.OutOffset = 3,					// 输出偏移：电机有小死区，小 PWM 推不动轮子，这里补一点让它一动就能起步

	.ErrorIntMax = 600,				// 积分上限，防止积分项无限增长
	.ErrorIntMin = -600,			// 积分下限
};

/* 速度环：外环，50ms 一次，负责把车"按住在原地"
   它的输出不是 PWM，而是角度环的目标值，也就是"该往前倾还是往后仰" */
PID_t SpeedPID = {
	.Kp = 2,						// 比例项：有速度就立刻让车身反向倾斜去刹车
	.Ki = 0.01,						// 积分项：慢慢累积速度误差，把车一点点拉回原位
	.Kd = 0,						// 微分项：速度环用不到微分

	.OutMax = 30,					// 输出上限，即角度目标最多偏多少度
	.OutMin = -30,					// 输出下限

	.ErrorIntMax = 200,				// 积分上限
	.ErrorIntMin = -200,			// 积分下限
};

/* 转向环：50ms 一次，用左右轮的速度差控制转弯
   它的输出直接变成差分 PWM，让一侧快一侧慢 */
PID_t TurnPID = {
	.Kp = 0,						// 比例项，为 0 时转向几乎没有响应，需要转弯要把它调到 1 以上
	.Ki = 0,						// 积分项
	.Kd = 1,						// 微分项

	.OutMax = 50,					// 输出上限
	.OutMin = -50,					// 输出下限

	.ErrorIntMax = 20,				// 积分上限
	.ErrorIntMin = -20,				// 积分下限
};

/* ==================== 主函数 ==================== */

int main(void)
{
	/* 外设初始化，顺序无所谓，但都要在定时器启动之前完成 */
	OLED_Init();		// OLED，用于显示各环参数
	MPU6050_Init();		// 六轴传感器，姿态数据的来源
	HC05_Init();		// 蓝牙遥控，接在 PA2/PA3（USART2），接收行车指令
	LED_Init();			// 板载 LED，指示运行状态
	Key_Init();			// 两个按键
	Motor_Init();		// 电机 GPIO 与底层 PWM
	Encoder_Init();		// 编码器，用来测速
	Serial_Init();		// USART1，调试串口

	Timer_Init();		// 1ms 定时中断，所有实时任务都在这个中断里跑

	while (1)
	{
		/* LED 亮灭直接反映 RunFlag，一眼就能看出程序是否在控制 */
		if (RunFlag) {LED_ON();} else {LED_OFF();}

		/* ---- 按键处理 ---- */
		KeyNum = Key_GetNum();
		if (KeyNum == 1)					// K1 用来启动/停止平衡程序
		{
			if (RunFlag == 0)				// 当前是停止状态，准备启动
			{
				/* 启动前检查车身姿态：倾斜超过 5 度就不许启动
				   否则一开环角度误差就很大，PWM 直接打满，车会窜出去 */
				if (Angle > -5 && Angle < 5)
				{
					/* 启动前清空三个环的历史状态（上次的误差、积分、输出），
					   避免上一轮残留的积分让这次启动瞬间猛冲 */
					PID_Init(&AnglePID);
					PID_Init(&SpeedPID);
					PID_Init(&TurnPID);
					RunFlag = 1;
				}
			}
			else							// 当前是运行状态，按下即停止
			{
				RunFlag = 0;
			}
		}

		/* ---- OLED 刷新 ---- */
		/* 三列并排显示：左列角度环、中列速度环、右列转向环，一行一个量 */
		OLED_Clear();
		OLED_Printf(0, 0, OLED_6X8, "  Angle");						// 左列标题
		OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", AnglePID.Kp);		// 角度环 Kp
		OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", AnglePID.Ki);		// 角度环 Ki
		OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", AnglePID.Kd);		// 角度环 Kd
		OLED_Printf(0, 32, OLED_6X8, "T:%+05.1f", AnglePID.Target);	// 角度环目标角度
		OLED_Printf(0, 40, OLED_6X8, "A:%+05.1f", Angle);			// 当前实测角度
		OLED_Printf(0, 48, OLED_6X8, "O:%+05.0f", AnglePID.Out);	// 角度环输出
		OLED_Printf(0, 56, OLED_6X8, "GY:%+05d", GY);				// 陀螺仪 Y 轴原始值，用来校零漂
		OLED_Printf(50, 0, OLED_6X8, "Speed");						// 中列标题
		OLED_Printf(50, 8, OLED_6X8, "%05.2f", SpeedPID.Kp);		// 速度环 Kp
		OLED_Printf(50, 16, OLED_6X8, "%05.2f", SpeedPID.Ki);		// 速度环 Ki
		OLED_Printf(50, 24, OLED_6X8, "%05.2f", SpeedPID.Kd);		// 速度环 Kd
		OLED_Printf(50, 32, OLED_6X8, "%+05.1f", SpeedPID.Target);	// 速度环目标速度
		OLED_Printf(50, 40, OLED_6X8, "%+05.1f", AveSpeed);			// 当前实测平均速度
		OLED_Printf(50, 48, OLED_6X8, "%+05.0f", SpeedPID.Out);		// 速度环输出（即角度目标）
		OLED_Printf(88, 0, OLED_6X8, "Turn");						// 右列标题
		OLED_Printf(88, 8, OLED_6X8, "%05.2f", TurnPID.Kp);			// 转向环 Kp
		OLED_Printf(88, 16, OLED_6X8, "%05.2f", TurnPID.Ki);		// 转向环 Ki
		OLED_Printf(88, 24, OLED_6X8, "%05.2f", TurnPID.Kd);		// 转向环 Kd
		OLED_Printf(88, 32, OLED_6X8, "%+05.1f", TurnPID.Target);	// 转向环目标差速
		OLED_Printf(88, 40, OLED_6X8, "%+05.1f", DifSpeed);			// 当前实测差速
		OLED_Printf(88, 48, OLED_6X8, "%+05.0f", TurnPID.Out);		// 转向环输出（即差分 PWM）

		/* ---- 蓝牙通信诊断，排查遥控问题时看最后一行 ---- */
		/* R 是串口收到的字节总数，C 是成功解析的指令条数
		   R 不涨：串口一个字节都没收到，查接线（HC-05 的 TXD 接 PA3、RXD 接 PA2）和波特率
		   R 涨但 C 不涨：收到了数据但认不出指令，查发送格式
		   R、C 都涨但中列 T 还是 0：指令没问题，是没按 K1 启动 */
		OLED_Printf(50, 56, OLED_6X8, "R%04d", HC05_RxCount);		// 收到的字节数
		OLED_Printf(88, 56, OLED_6X8, "C%04d", HC05_CmdCount);		// 解析成功的指令数

		OLED_Update();

		/* 蓝牙指令的接收和解析全部在 HC05 模块里完成（串口中断收字符，
		   1ms 中断里断句并解析），主循环这里不需要再处理，只负责显示 */
	}
}

/* ==================== 1ms 定时中断 ==================== */
/* 平衡车的实时性全靠这个中断：读姿态、算角度、跑 PID、输出 PWM 都在这里完成 */
void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count0, Count1;		// 两个软件分频计数器

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		/* 一进中断就清标志位。这样如果在退出前标志位又被置 1，
		   就说明本次中断执行时间已经超过了 1ms，实时性被破坏了 */
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

		Key_Tick();				// 按键扫描依赖 1ms 节拍做消抖
		HC05_Tick();			// 蓝牙指令断句解析 + 失联检测

		/* ---- 10ms 分支：姿态解算 + 角度环 ---- */
		Count0++;
		if (Count0 >= 10)
		{
			Count0 = 0;

			/* 在中断里读传感器，间隔严格 10ms，角度积分才不会忽快忽慢 */
			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

			/* 陀螺仪 Y 轴零漂校准：静止时读数不为 0 是器件特性，必须扣掉
			   做法是静止时看 OLED 上的 GY，把显示值加上这里的 46 就是原始零漂 */
			GY -= 46;

			/* 加速度计解算倾角：atan2 得到弧度，乘 180/π 转成角度 */
			AngleAcc = -atan2(AX, AZ) / 3.14159 * 180;

			/* 机械平衡点校准：车架的零位和传感器零位并不重合，这里补一个固定偏差
			   标定方法：不按 K1，用手扶住车身，找到能自然停住的那个姿态，
			   读 OLED 上的 A 值，A 显示多少，就把这里的补偿改成它的相反数，
			   让机械平衡点处 A 显示 0。这个值偏小，车会以为平衡点在前方，一直往前冲 */
			AngleAcc += 1;

			/* 陀螺仪积分：32768 是 int16 满量程，2000 是配置的 ±2000°/s 量程，
			   0.01 是积分周期。注意要在上次滤波结果 Angle 上累加，而不是 AngleAcc */
			AngleGyro = Angle + GY / 32768.0 * 2000 * 0.01;

			/* 互补滤波：加速度计准但抖，陀螺仪稳但漂，按比例融合
			   Alpha 越大越信加速度计，越小越信陀螺仪 */
			float Alpha = 0.01;
			Angle = Alpha * AngleAcc + (1 - Alpha) * AngleGyro;

			/* 倒地保护：角度超过 ±50 度认为车已经翻了，直接停掉平衡程序 */
			if (Angle > 50 || Angle < -50)
			{
				RunFlag = 0;
			}

			/* ---- 角度环：唯一的执行环节，所有控制最终都汇到这里 ---- */
			if (RunFlag)
			{
				AnglePID.Actual = Angle;		// 反馈量是当前角度
				PID_Update(&AnglePID);			// 算出一个应该施加的"扶正力度"

				/* 角度环输出取负号才是 PWM：车往前倒（角度为正）时，轮子必须往前追，
				   这个符号由电机的安装方向决定，写反了车会直接倒下去 */
				AvePWM = -AnglePID.Out;

				/* 平均 PWM 负责前后，差分 PWM 负责左右，两轮叠加/相减得到各自的 PWM */
				LeftPWM = AvePWM + DifPWM / 2;
				RightPWM = AvePWM - DifPWM / 2;

				/* 限幅：叠加后可能超出电机允许范围，卡在 -100~100 之内 */
				if (LeftPWM > 100) {LeftPWM = 100;} else if (LeftPWM < -100) {LeftPWM = -100;}
				if (RightPWM > 100) {RightPWM = 100;} else if (RightPWM < -100) {RightPWM = -100;}

				Motor_SetPWM(1, LeftPWM);		// 左轮
				Motor_SetPWM(2, RightPWM);		// 右轮
			}
			else
			{
				/* 程序停止时必须主动把 PWM 清零，否则电机会保持上一次的占空比继续转 */
				Motor_SetPWM(1, 0);
				Motor_SetPWM(2, 0);
			}
		}

		/* ---- 50ms 分支：测速 + 速度环 + 转向环 ---- */
		Count1++;
		if (Count1 >= 50)
		{
			Count1 = 0;

			/* 编码器磁铁转一圈计 44 个数，读取周期 0.05s，减速箱减速比 9.27666，
			   一路除下来得到电机输出轴的转速，单位是转/秒 */
			LeftSpeed = Encoder_Get(1) / 44.0 / 0.05 / 9.27666;
			RightSpeed = Encoder_Get(2) / 44.0 / 0.05 / 9.27666;

			/* 两轮合成平均速度（决定前后），分解出差分速度（决定转弯） */
			AveSpeed = (LeftSpeed + RightSpeed) / 2.0;
			DifSpeed = LeftSpeed - RightSpeed;

			/* 速度死区：编码器静止时也有微小噪声，太小的速度直接当 0，
			   否则速度环会被噪声推着乱动 */
			if (AveSpeed > -0.2 && AveSpeed < 0.2) {AveSpeed = 0;}
			if (DifSpeed > -0.2 && DifSpeed < 0.2) {DifSpeed = 0;}

			if (RunFlag)
			{
				/* 前后和转向的目标值全部由遥控模块给出。
				   没收到任何指令时这两个值都是 0，车就原地保持平衡；
				   链路超时断开会自动清零，相当于失联后自动刹车 */
				SpeedPID.Target = HC05_Speed;
				TurnPID.Target = HC05_Turn;

				/* 速度环：输出的不是 PWM，而是角度环的目标角度，
				   这就是串级 PID 的关键——用"倾斜"去换"速度" */
				SpeedPID.Actual = AveSpeed;
				PID_Update(&SpeedPID);
				AnglePID.Target = SpeedPID.Out;

				/* 转向环：反馈量是左右轮速度差，输出直接变成差分 PWM */
				TurnPID.Actual = DifSpeed;
				PID_Update(&TurnPID);
				DifPWM = TurnPID.Out;
			}
		}

		/* 退出前再查一次标志位：若又被置 1，说明本次中断超时了 */
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			TimerErrorFlag = 1;

			/* 必须清掉，否则中断会反复重入，主循环一行都跑不到 */
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		}

		/* 记下中断退出时刻的计数值，用于评估中断耗时 */
		TimerCount = TIM_GetCounter(TIM1);
	}
}