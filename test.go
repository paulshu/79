package main

import (
	"C"
	"math"
)

func Round(f float64, n int) float64 {
	pow10_n := math.Pow10(n)
	return math.Trunc((f+0.5/pow10_n)*pow10_n) / pow10_n
}

// 按n位四舍五入

func Rounddown(f float64, n int) float64 {
	pow10_n := math.Pow10(n)
	return math.Trunc(f*pow10_n) / pow10_n
}

// 去掉n位后的尾数
func ExtremumInArray(array []float64) (float64, float64) {
	if len(array) < 1 {
		return 0.0, 0.0
	}
	min := array[0]
	max := array[0]
	for _, v := range array {
		if v < min {
			min = v
		} else if v > max {
			max = v
		}
	}
	return min, max
}

// 求数组的最大值和最小值

//export Formula_calculation
func Formula_calculation(a_x, a_y, a_z float64) (float64, float64) {
	var vector_og_on_xz, vector_ob_on_xz, vector_ob_on_xz_x_angle, vector_og_on_xz_x_angle, o_centre_gravity_x, o_gate_b1_x, o_gate_b1_z float64
	var vector_ob1_length_x, vector_ob1_length_z, vector_ab1_length_x, vector_ab1_length_y, vector_ab1_length_z, vector_ab_length_x, vector_ab_length_y, vector_ab_length_z, vector_ab1_ob1_on_xz_angle, od_length, cd_length, car_climbing_angle, vector_ob_slope, vector_ab_slope, vector_ob_ab_angle float64
	var vector_ab_xz_angle, vector_ab1_xz_angle, od_pole_force_arm, cd_pole_force_arm, od_min_force, cd_max_force, spring_cd_length, spring_od_length, initial_mean_diameter, wire_diameter_d, mean_diameter float64
	var no_solid_position_active_coil_num, active_coil_num, spring_rate, theoretical_free_length, free_length, spring_pitch, spring_helix_angle float64

	var open_angle_arr_pre, open_angle_arr, o_gate_b_x_arr, o_gate_b_y_arr, o_gate_b_z_arr, pole_length_arr, motor_output_speed_arr, motor_pwm_arr, vector_ba_length_x_arr, vector_ba_length_z_arr, vector_ob_length_x_arr, vector_ob_length_z_arr, vector_ba_ob_on_xz_angle_arr, pole_force_arm_arr, vector_ab_xz_angle_arr []float64
	var uphill_gravity_arm_arr, flat_slope_gravity_arm_arr, downhill_gravity_arm_arr, uphill_gravity_torque_arr, flat_slope_gravity_torque_arr, downhill_gravity_torque_arr, uphill_gravity_torque_snow_arr, flat_slope_gravity_torque_snow_arr, downhill_gravity_torque_snow_arr, nt_spring_force_arr, nt_lower_deviation_spring_force_arr []float64
	var nt_upper_deviation_spring_force_arr, nt_after_life_spring_force_arr, nt_lower_deviation_spring_torque_arr, nt_median_spring_torque_arr, double_nt_median_spring_torque_arr, nt_upper_deviation_spring_torque_arr, nt_after_life_spring_torque_arr, ht_lower_deviation_spring_torque_arr, ht_median_spring_torque_arr, ht_upper_deviation_spring_torque_arr []float64
	var ht_after_life_spring_torque_arr, nt_open_dynamic_friction_arr, lt_open_dynamic_friction_arr, ht_open_dynamic_friction_arr, nt_close_dynamic_friction_arr, lt_close_dynamic_friction_arr, ht_close_dynamic_friction_arr, nt_open_static_friction_arr, lt_open_static_friction_arr, ht_open_static_friction_arr, nt_close_static_friction_arr, lt_close_static_friction_arr, ht_close_static_friction_arr []float64
	var uphill_ht_lower_deviation_hover_arr, uphill_ht_median_hover_arr, uphill_ht_upper_deviation_hover_arr, uphill_ht_after_life_hover_arr, flat_slope_ht_lower_deviation_hover_arr, flat_slope_ht_median_hover_arr, flat_slope_ht_upper_deviation_hover_arr, flat_slope_ht_after_life_hover_arr, downhill_ht_lower_deviation_hover_arr, downhill_ht_median_hover_arr, downhill_ht_upper_deviation_hover_arr, downhill_ht_after_life_hover_arr []float64

	const PI = 3.141592653589793
	const Initial_wire_diameter = 3.6 // 弹簧初选线径
	var k, c, f1, f2 float64
	var oa, oaf int
	centre_gravity_x := 3850.0
	centre_gravity_z := 853.576
	hinge_x := 3470.5
	// hinge_y := 0.0
	hinge_z := 1240.0
	open_angle := 89.0
	door_weight := 32.31
	climbing_degree := 0.2
	b_x := 3711.0
	b_y := -659.67
	b_z := 745.25
	open_time := 6.0
	snow_load := 5.0

	platform_spring_static_length := 224.1
	// platform_max_tensile_strength := 2100.0
	platform_s_elastic_modulus := 78800.0
	platform_inside_diameter := 21.55
	platform_lead := 12.5
	platform_gear_transmission_ratio := 22.2
	platform_rated_speed := 7500.0
	flocking := 0.4
	free_length_filter := 1.0
	open_angle_filter := 0
	open_dynamic_friction := 250.0
	close_dynamic_friction := -270.0
	open_static_friction := 300.0
	close_static_friction := -320.0

	vector_og_on_xz = math.Sqrt((centre_gravity_x-hinge_x)*(centre_gravity_x-hinge_x) + (centre_gravity_z-hinge_z)*(centre_gravity_z-hinge_z))
	// 向量OG在XZ平面的投影长度
	vector_ob_on_xz = math.Sqrt(math.Pow((b_x-hinge_x), 2) + math.Pow((b_z-hinge_z), 2))
	// 向量OB在XZ平面的投影长度
	// vector_ab = math.Sqrt((a_x-b_x)*(a_x-b_x) + (a_y-b_y)*(a_y-b_y) + (a_z-b_z)*(a_z-b_z))
	// 向量AB的长度,即撑杆关门长度
	vector_ob_on_xz_x_angle = (180 / PI) * math.Atan((b_z-hinge_z)/(b_x-hinge_x))
	// 向量OB在XZ平面与X轴的夹角（°）
	vector_og_on_xz_x_angle = (180 / PI) * math.Atan((centre_gravity_z-hinge_z)/(centre_gravity_x-hinge_x))
	// 向量OG在XZ平面与X轴的夹角（°）
	o_centre_gravity_x = vector_og_on_xz*math.Cos((open_angle-math.Abs(vector_og_on_xz_x_angle))*PI/180) + hinge_x
	// 开门重心X坐标
	// o_centre_gravity_z = vector_og_on_xz*math.Sin((open_angle-math.Abs(vector_og_on_xz_x_angle))*PI/180) + hinge_z
	// 开门重心z坐标

	o_gate_b1_x = vector_ob_on_xz*math.Cos((open_angle-math.Abs(vector_ob_on_xz_x_angle))*PI/180) + hinge_x
	// 开门尾门连接点B1 X坐标
	o_gate_b1_z = vector_ob_on_xz*math.Sin((open_angle-math.Abs(vector_ob_on_xz_x_angle))*PI/180) + hinge_z
	// 开门尾门连接点B1 z坐标
	vector_ob1_length_x = o_gate_b1_x - hinge_x
	// 向量OB1的长度（X方向分量）

	// vector_ob1_length_y = math.Abs(b_y - hinge_y)
	// 向量OB1的长度（Y方向分量）
	vector_ob1_length_z = o_gate_b1_z - hinge_z
	// 向量OB1的长度（Z方向分量）

	vector_ab1_length_x = o_gate_b1_x - a_x
	// 向量AB1的长度（X方向分量）
	vector_ab1_length_y = math.Abs(b_y - a_y)
	// 向量AB1的长度（Y方向分量）

	vector_ab1_length_z = o_gate_b1_z - a_z
	// 向量AB1的长度（Z方向分量）

	vector_ab_length_x = b_x - a_x
	// 向量AB的长度（X方向分量）
	vector_ab_length_y = b_y - a_y
	// 向量AB的长度（Y方向分量）
	vector_ab_length_z = b_z - a_z
	// 向量AB的长度（Z方向分量）

	vector_ab1_ob1_on_xz_angle = (180 / PI) * math.Acos((vector_ob1_length_x*vector_ab1_length_x+vector_ob1_length_z*vector_ab1_length_z)/(math.Sqrt(vector_ob1_length_x*vector_ob1_length_x+vector_ob1_length_z*vector_ob1_length_z)*math.Sqrt(vector_ab1_length_x*vector_ab1_length_x+vector_ab1_length_z*vector_ab1_length_z)))
	// 向量AB1和OB1在XZ平面的夹角（°）
	od_length = math.Sqrt(math.Pow((o_gate_b1_x-a_x), 2) + math.Pow((b_y-a_y), 2) + math.Pow((o_gate_b1_z-a_z), 2))
	// 撑杆开门长度
	cd_length = math.Sqrt(math.Pow((b_x-a_x), 2) + math.Pow((b_y-a_y), 2) + math.Pow((b_z-a_z), 2))
	// 撑杆关门长度
	// working_stroke = od_length - cd_length                     // 撑杆工作行程
	car_climbing_angle = math.Atan(climbing_degree) * 180 / PI // 汽车爬坡角度

	vector_ob_slope = (b_z - hinge_z) / (b_x - hinge_x) // 向量ob的斜率
	vector_ab_slope = (b_z - a_z) / (b_x - a_x)         // 向量ab的斜率
	vector_ob_ab_angle = (180 / PI) * math.Atan((vector_ab_slope-vector_ob_slope)/(1+vector_ab_slope*vector_ob_slope))
	// 向量OB和向量AB的夹角

	vector_ab_xz_angle = math.Acos(math.Sqrt(math.Pow(vector_ab_length_x, 2)+math.Pow(vector_ab_length_z, 2))/math.Sqrt(math.Pow(vector_ab_length_x, 2)+math.Pow(vector_ab_length_y, 2)+math.Pow(vector_ab_length_z, 2))) * 180 / PI
	// 向量AB与XZ平面的线面夹角（空间夹角）
	vector_ab1_xz_angle = math.Acos(math.Sqrt(math.Pow(vector_ab1_length_x, 2)+math.Pow(vector_ab1_length_z, 2))/math.Sqrt(math.Pow(vector_ab1_length_x, 2)+math.Pow(vector_ab1_length_y, 2)+math.Pow(vector_ab1_length_z, 2))) * 180 / PI
	// 向量AB1与XZ平面的线面夹角（空间夹角）

	od_pole_force_arm = vector_ob_on_xz * math.Sin(vector_ab1_ob1_on_xz_angle*PI/180)
	// 开门撑杆力臂
	cd_pole_force_arm = vector_ob_on_xz * math.Sin(vector_ob_ab_angle*PI/180)
	// 关门撑杆力臂
	od_min_force = door_weight * 9.8 * (o_centre_gravity_x - hinge_x) / (2 * od_pole_force_arm) / math.Cos(vector_ab1_xz_angle*PI/180)
	// 开门在最高点需求力
	cd_max_force = door_weight * 9.8 * (centre_gravity_x - hinge_x) / (2 * cd_pole_force_arm) / math.Cos(vector_ab_xz_angle*PI/180)
	// 关门到开门瞬间时需求力

	// 弹簧计算 //

	spring_cd_length = cd_length - platform_spring_static_length
	spring_od_length = od_length - platform_spring_static_length
	// deformation = spring_od_length - spring_cd_length
	// max_test_shear_stress = platform_max_tensile_strength * 0.6                // 最大试验切应用力
	// dynamic_load_allowable_shear_stress = platform_max_tensile_strength * 0.57 // 动负荷许用切应力
	initial_mean_diameter = platform_inside_diameter + Initial_wire_diameter // 弹簧初选中径
	// spring_theoretical_rate = (cd_max_force - od_min_force) / deformation    // 理论刚度

	// 理论线径
	c = initial_mean_diameter / Initial_wire_diameter
	k = (4*c-1)/(4*c-4) + (0.615 / c)
	// theoretical_wire_diameter = math.Cbrt(8 * k * initial_mean_diameter * cd_max_force / (PI * dynamic_load_allowable_shear_stress))
	// 弹簧线径
	wire_diameter_d = Round(math.Cbrt((8*k*initial_mean_diameter*cd_max_force)/(PI*1470)), 1)
	// 其中1470为现在与供应商确定的最大工作切应力

	if wire_diameter_d > 4.0 {
		wire_diameter_d = 4.0
	}

	if wire_diameter_d < 3.5 {
		wire_diameter_d = 3.5
	}
	// 弹簧中径
	mean_diameter = platform_inside_diameter + wire_diameter_d

	// 理论有效圈数
	// theoretical_active_coil_num = platform_s_elastic_modulus * math.Pow(wire_diameter_d, 4) / (8 * math.Pow(mean_diameter, 3) * spring_theoretical_rate)

	// 不压并建议有效圈数
	no_solid_position_active_coil_num = (spring_cd_length-13)/(wire_diameter_d+flocking) - 2
	// 代入有效圈数
	active_coil_num = Rounddown(no_solid_position_active_coil_num, 1)

	// 实际刚度
	spring_rate = platform_s_elastic_modulus * math.Pow(wire_diameter_d, 4) / (8 * math.Pow(mean_diameter, 3) * active_coil_num)

	// 高温弹簧刚度
	// ht_spring_rate = spring_rate * 0.96

	// 理论自由长度

	f1 = od_min_force / spring_rate
	f2 = cd_max_force / spring_rate
	theoretical_free_length = math.Min((spring_od_length + f1), (spring_cd_length + f2))

	free_length = Round(theoretical_free_length, 1) * free_length_filter // 自由长度

	// 弹簧节距
	spring_pitch = (free_length - 1.5*wire_diameter_d) / active_coil_num

	// 弹簧螺旋升角
	spring_helix_angle = Round((math.Atan(spring_pitch/(PI*mean_diameter)) * 180 / PI), 3)
	if spring_helix_angle >= 9.05 && wire_diameter_d < 3.8 {
		wire_diameter_d = Round((wire_diameter_d + 0.1), 1)
		// 弹簧中径
		mean_diameter = platform_inside_diameter + wire_diameter_d

		// 理论有效圈数
		// theoretical_active_coil_num = platform_s_elastic_modulus * math.Pow(wire_diameter_d, 4) / (8 * math.Pow(mean_diameter, 3) * spring_theoretical_rate)

		// 不压并建议有效圈数
		no_solid_position_active_coil_num = (spring_cd_length-13)/(wire_diameter_d+flocking) - 2
		// 代入有效圈数
		active_coil_num = Rounddown(no_solid_position_active_coil_num, 1)

		// 实际刚度
		spring_rate = platform_s_elastic_modulus * math.Pow(wire_diameter_d, 4) / (8 * math.Pow(mean_diameter, 3) * active_coil_num)

		// 高温弹簧刚度
		// ht_spring_rate = spring_rate * 0.96

		// 理论自由长度

		f1 = od_min_force / spring_rate
		f2 = cd_max_force / spring_rate
		theoretical_free_length = math.Min((spring_od_length + f1), (spring_cd_length + f2))

		free_length = Round(theoretical_free_length, 1) * free_length_filter // 自由长度
	}

	// 弹簧总圈数
	// total_num = active_coil_num + 2

	// spring_od_force = Round((free_length-spring_od_length)*spring_rate, 3)
	// spring_cd_force = Round((free_length-spring_cd_length)*spring_rate, 3)

	// 撑杆中间参数计算 //

	// 计算角度增量oa
	if math.Mod(open_angle, 2) == 0 {
		oa = int(open_angle)/2 + 1
	} else {
		oa = int(open_angle)/2 + 2
	}
	oaf = open_angle_filter / 2

	//开门角度增量

	sum := 0
	for i := 0; i < oa; i++ {
		sum = i * 2
		open_angle_arr_pre = append(open_angle_arr_pre, float64(sum))
	}
	open_angle_arr = open_angle_arr_pre[oaf:oa]

	// 开门尾门连接点B实时 X坐标
	for _, v := range open_angle_arr {
		o_gate_b_x_arr = append(o_gate_b_x_arr, (vector_ob_on_xz*math.Cos((v+vector_ob_on_xz_x_angle)*PI/180) + hinge_x))
	}

	// 开门尾门连接点B实时 y坐标
	for i := 0; i < oa; i++ {
		o_gate_b_y_arr = append(o_gate_b_y_arr, b_y)
	}
	// 开门尾门连接点B实时 z坐标
	for _, v := range open_angle_arr {
		o_gate_b_z_arr = append(o_gate_b_z_arr, vector_ob_on_xz*math.Sin((v+vector_ob_on_xz_x_angle)*PI/180)+hinge_z)
	}

	// 撑杆实时长度
	for i := 0; i < oa; i++ {
		pole_length_arr = append(pole_length_arr, math.Sqrt(math.Pow((o_gate_b_x_arr[i]-a_x), 2)+math.Pow((o_gate_b_y_arr[i]-a_y), 2)+math.Pow((o_gate_b_z_arr[i]-a_z), 2)))
	}

	// 电机输出转速（r/min） ,假定角度增量为2度
	pla1 := pole_length_arr[1:oa]
	pla2 := pole_length_arr[0 : oa-1]
	motor_output_speed_arr = append(motor_output_speed_arr, 2500) // 此处需要将常数修改为变量
	for i, v := range pla1 {
		motor_output_speed_arr = append(motor_output_speed_arr, (v-pla2[i])*60/(open_time/open_angle*2*platform_lead)*platform_gear_transmission_ratio)
	}

	// PWM调制占空比
	motor_pwm_arr = append(motor_pwm_arr, 0.33333333)
	mos := motor_output_speed_arr[1:oa]
	for _, v := range mos {
		motor_pwm_arr = append(motor_pwm_arr, v/platform_rated_speed)
	}

	// 向量BA的实时长度（X方向分量）
	for _, v := range o_gate_b_x_arr {
		vector_ba_length_x_arr = append(vector_ba_length_x_arr, a_x-v)
	}

	// 向量BA的实时长度（Z方向分量）
	for _, v := range o_gate_b_z_arr {
		vector_ba_length_z_arr = append(vector_ba_length_z_arr, a_z-v)
	}

	// 向量OB的实时长度（X方向分量）
	for _, v := range o_gate_b_x_arr {
		vector_ob_length_x_arr = append(vector_ob_length_x_arr, hinge_x-v)
	}

	// 向量OB的实时长度（Z方向分量）
	for _, v := range o_gate_b_z_arr {
		vector_ob_length_z_arr = append(vector_ob_length_z_arr, hinge_z-v)
	}

	// 向量BA和OB在XZ平面的夹角(°)
	for i, v := range vector_ob_length_x_arr {
		vector_ba_ob_on_xz_angle_arr = append(vector_ba_ob_on_xz_angle_arr, 180/PI*math.Acos((v*vector_ba_length_x_arr[i]+vector_ob_length_z_arr[i]*vector_ba_length_z_arr[i])/
			(math.Sqrt(math.Pow(v, 2)+math.Pow(vector_ob_length_z_arr[i], 2))*math.Sqrt(math.Pow(vector_ba_length_x_arr[i], 2)+math.Pow(vector_ba_length_z_arr[i], 2)))))
	}

	// 撑杆力臂
	for _, v := range vector_ba_ob_on_xz_angle_arr {
		pole_force_arm_arr = append(pole_force_arm_arr, vector_ob_on_xz*math.Sin(v*PI/180))
	}

	// 向量AB与XZ平面的实时线面夹角（空间夹角）
	for i, v := range vector_ba_length_x_arr {
		vector_ab_xz_angle_arr = append(vector_ab_xz_angle_arr, 180/PI*math.Acos((v*v+math.Pow(vector_ba_length_z_arr[i], 2))/
			(math.Sqrt(math.Pow(v, 2)+math.Pow(vector_ab_length_y, 2)+math.Pow(vector_ba_length_z_arr[i], 2))*math.Sqrt(math.Pow(v, 2)+math.Pow(vector_ba_length_z_arr[i], 2)))))
	}

	// 上坡重力力臂
	for _, v := range open_angle_arr {
		uphill_gravity_arm_arr = append(uphill_gravity_arm_arr, vector_og_on_xz*math.Cos((vector_og_on_xz_x_angle+v-car_climbing_angle)*PI/180))
	}

	// 平坡重力力臂
	for _, v := range open_angle_arr {
		flat_slope_gravity_arm_arr = append(flat_slope_gravity_arm_arr, vector_og_on_xz*math.Cos((vector_og_on_xz_x_angle+v)*PI/180))
	}

	// 下坡重力力臂
	for _, v := range open_angle_arr {
		downhill_gravity_arm_arr = append(downhill_gravity_arm_arr, vector_og_on_xz*math.Cos((vector_og_on_xz_x_angle+v+car_climbing_angle)*PI/180))
	}

	//  重力矩 //

	// 上坡重力力矩
	for _, v := range uphill_gravity_arm_arr {
		uphill_gravity_torque_arr = append(uphill_gravity_torque_arr, math.Abs(door_weight*9.8*v/1000))
	}

	// 平坡重力力矩
	for _, v := range flat_slope_gravity_arm_arr {
		flat_slope_gravity_torque_arr = append(flat_slope_gravity_torque_arr, math.Abs(door_weight*9.8*v/1000))
	}

	// 下坡重力力矩
	for _, v := range downhill_gravity_arm_arr {
		downhill_gravity_torque_arr = append(downhill_gravity_torque_arr, math.Abs(door_weight*9.8*v/1000))
	}

	//  雪载重力矩 //

	// 上坡雪载重力力矩
	for _, v := range uphill_gravity_arm_arr {
		uphill_gravity_torque_snow_arr = append(uphill_gravity_torque_snow_arr, math.Abs((door_weight+snow_load)*9.8*v/1000))
	}

	// 平坡雪载重力力矩
	for _, v := range flat_slope_gravity_arm_arr {
		flat_slope_gravity_torque_snow_arr = append(flat_slope_gravity_torque_snow_arr, math.Abs((door_weight+snow_load)*9.8*v/1000))
	}

	// 下坡雪载重力力矩
	for _, v := range downhill_gravity_arm_arr {
		downhill_gravity_torque_snow_arr = append(downhill_gravity_torque_snow_arr, math.Abs((door_weight+snow_load)*9.8*v/1000))
	}

	//  弹簧力矩 以下均为单根力矩 //

	// 常温弹簧力中值
	for _, v := range pole_length_arr {
		nt_spring_force_arr = append(nt_spring_force_arr, (free_length-(spring_cd_length+v-cd_length))*spring_rate)
	}

	// 常温弹簧力下偏差（最小值）
	for _, v := range nt_spring_force_arr {
		nt_lower_deviation_spring_force_arr = append(nt_lower_deviation_spring_force_arr, (v - 30))
	}

	// 常温弹簧力上偏差（最大值）
	for _, v := range nt_spring_force_arr {
		nt_upper_deviation_spring_force_arr = append(nt_upper_deviation_spring_force_arr, (v + 30))
	}

	// 常温弹簧力寿命后
	for _, v := range nt_spring_force_arr {
		nt_after_life_spring_force_arr = append(nt_after_life_spring_force_arr, v*(1-0.05))
	}

	// 常温下偏差弹簧力矩
	for i, v := range nt_lower_deviation_spring_force_arr {
		nt_lower_deviation_spring_torque_arr = append(nt_lower_deviation_spring_torque_arr, v*pole_force_arm_arr[i]*math.Cos(PI/180*vector_ab_xz_angle_arr[i])/1000)
	}

	// 常温中值弹簧力矩
	for i, v := range nt_spring_force_arr {
		nt_median_spring_torque_arr = append(nt_median_spring_torque_arr, v*pole_force_arm_arr[i]*math.Cos(PI/180*vector_ab_xz_angle_arr[i])/1000)
	}

	// 常温中值两根弹簧力矩
	for i, v := range nt_spring_force_arr {
		double_nt_median_spring_torque_arr = append(double_nt_median_spring_torque_arr, 2*v*pole_force_arm_arr[i]*math.Cos(PI/180*vector_ab_xz_angle_arr[i])/1000)
	}

	// 常温上偏差弹簧力矩
	for i, v := range nt_upper_deviation_spring_force_arr {
		nt_upper_deviation_spring_torque_arr = append(nt_upper_deviation_spring_torque_arr, v*pole_force_arm_arr[i]*math.Cos(PI/180*vector_ab_xz_angle_arr[i])/1000)
	}

	// 常温寿命后弹簧力矩
	for i, v := range nt_after_life_spring_force_arr {
		nt_after_life_spring_torque_arr = append(nt_after_life_spring_torque_arr, v*pole_force_arm_arr[i]*math.Cos(PI/180*vector_ab_xz_angle_arr[i])/1000)
	}

	// 高温下偏差弹簧力矩
	for _, v := range nt_lower_deviation_spring_torque_arr {
		ht_lower_deviation_spring_torque_arr = append(ht_lower_deviation_spring_torque_arr, v*0.96)
	}

	// 高温中值弹簧力矩
	for _, v := range nt_median_spring_torque_arr {
		ht_median_spring_torque_arr = append(ht_median_spring_torque_arr, v*0.96)
	}

	// 高温上偏差弹簧力矩
	for _, v := range nt_upper_deviation_spring_torque_arr {
		ht_upper_deviation_spring_torque_arr = append(ht_upper_deviation_spring_torque_arr, v*0.96)
	}

	// 高温寿命后弹簧力矩
	for _, v := range nt_after_life_spring_torque_arr {
		ht_after_life_spring_torque_arr = append(ht_after_life_spring_torque_arr, v*0.96)
	}

	// 撑杆动摩擦阻力 //

	// 常温开门摩擦阻力
	for i := 0; i < oa; i++ {
		nt_open_dynamic_friction_arr = append(nt_open_dynamic_friction_arr, open_dynamic_friction)
	}

	// 低温开门摩擦阻力
	for i := 0; i < oa; i++ {
		lt_open_dynamic_friction_arr = append(lt_open_dynamic_friction_arr, open_dynamic_friction+50)
	}
	// 高温开门摩擦阻力
	for i := 0; i < oa; i++ {
		ht_open_dynamic_friction_arr = append(ht_open_dynamic_friction_arr, open_dynamic_friction-50)
	}
	// 常温关门摩擦阻力
	for i := 0; i < oa; i++ {
		nt_close_dynamic_friction_arr = append(nt_close_dynamic_friction_arr, close_dynamic_friction)
	}
	// 低温关门摩擦阻力
	for i := 0; i < oa; i++ {
		lt_close_dynamic_friction_arr = append(lt_close_dynamic_friction_arr, close_dynamic_friction-50)
	}
	// 高温关门摩擦阻力
	for i := 0; i < oa; i++ {
		ht_close_dynamic_friction_arr = append(ht_close_dynamic_friction_arr, close_dynamic_friction+50)
	}

	// 撑杆静摩擦阻力

	// 常温开门静摩擦阻力
	for i := 0; i < oa; i++ {
		nt_open_static_friction_arr = append(nt_open_static_friction_arr, open_static_friction)
	}
	// 低温开门静摩擦阻力
	for i := 0; i < oa; i++ {
		lt_open_static_friction_arr = append(lt_open_static_friction_arr, open_static_friction+50)
	}
	// 高温开门静摩擦阻力
	for i := 0; i < oa; i++ {
		ht_open_static_friction_arr = append(ht_open_static_friction_arr, open_static_friction-50)
	}
	// 常温关门静摩擦阻力
	for i := 0; i < oa; i++ {
		nt_close_static_friction_arr = append(nt_close_static_friction_arr, close_static_friction)
	}
	// 低温关门静摩擦阻力
	for i := 0; i < oa; i++ {
		lt_close_static_friction_arr = append(lt_close_static_friction_arr, close_static_friction-50)
	}
	// 高温关门静摩擦阻力
	for i := 0; i < oa; i++ {
		ht_close_static_friction_arr = append(ht_close_static_friction_arr, close_static_friction+50)
	}

	// 悬停计算  //

	// 高温

	// 上坡高温下偏差悬停
	for i, v := range uphill_gravity_torque_arr {
		uphill_ht_lower_deviation_hover_arr = append(uphill_ht_lower_deviation_hover_arr, (ht_lower_deviation_spring_torque_arr[i]-v/2)/pole_force_arm_arr[i]/math.Cos(PI/180*vector_ab_xz_angle_arr[i])*1000)
	}

	// 上坡高温中值悬停
	for i, v := range uphill_gravity_torque_arr {
		uphill_ht_median_hover_arr = append(uphill_ht_median_hover_arr, (ht_median_spring_torque_arr[i]-v/2)/pole_force_arm_arr[i]/math.Cos(PI/180*vector_ab_xz_angle_arr[i])*1000)
	}

	// 上坡高温上偏差悬停
	for i, v := range uphill_gravity_torque_arr {
		uphill_ht_upper_deviation_hover_arr = append(uphill_ht_upper_deviation_hover_arr, (ht_upper_deviation_spring_torque_arr[i]-v/2)/pole_force_arm_arr[i]/math.Cos(PI/180*vector_ab_xz_angle_arr[i])*1000)
	}

	// 上坡高温寿命后悬停
	for i, v := range uphill_gravity_torque_arr {
		uphill_ht_after_life_hover_arr = append(uphill_ht_after_life_hover_arr, (ht_after_life_spring_torque_arr[i]-v/2)/pole_force_arm_arr[i]/math.Cos(PI/180*vector_ab_xz_angle_arr[i])*1000)
	}

	// 平坡高温下偏差悬停
	for i, v := range flat_slope_gravity_torque_arr {
		flat_slope_ht_lower_deviation_hover_arr = append(flat_slope_ht_lower_deviation_hover_arr, (ht_lower_deviation_spring_torque_arr[i]-v/2)/pole_force_arm_arr[i]/math.Cos(PI/180*vector_ab_xz_angle_arr[i])*1000)
	}

	// 平坡高温中值悬停
	for i, v := range flat_slope_gravity_torque_arr {
		flat_slope_ht_median_hover_arr = append(flat_slope_ht_median_hover_arr, (ht_median_spring_torque_arr[i]-v/2)/pole_force_arm_arr[i]/math.Cos(PI/180*vector_ab_xz_angle_arr[i])*1000)
	}

	// 平坡高温上偏差悬停
	for i, v := range flat_slope_gravity_torque_arr {
		flat_slope_ht_upper_deviation_hover_arr = append(flat_slope_ht_upper_deviation_hover_arr, (ht_upper_deviation_spring_torque_arr[i]-v/2)/pole_force_arm_arr[i]/math.Cos(PI/180*vector_ab_xz_angle_arr[i])*1000)
	}

	// 平坡高温寿命后悬停
	for i, v := range flat_slope_gravity_torque_arr {
		flat_slope_ht_after_life_hover_arr = append(flat_slope_ht_after_life_hover_arr, (ht_after_life_spring_torque_arr[i]-v/2)/pole_force_arm_arr[i]/math.Cos(PI/180*vector_ab_xz_angle_arr[i])*1000)
	}

	// 下坡高温下偏差悬停
	for i, v := range downhill_gravity_torque_arr {
		downhill_ht_lower_deviation_hover_arr = append(downhill_ht_lower_deviation_hover_arr, (ht_lower_deviation_spring_torque_arr[i]-v/2)/pole_force_arm_arr[i]/math.Cos(PI/180*vector_ab_xz_angle_arr[i])*1000)
	}

	// 下坡高温中值悬停
	for i, v := range downhill_gravity_torque_arr {
		downhill_ht_median_hover_arr = append(downhill_ht_median_hover_arr, (ht_median_spring_torque_arr[i]-v/2)/pole_force_arm_arr[i]/math.Cos(PI/180*vector_ab_xz_angle_arr[i])*1000)
	}

	// 下坡高温上偏差悬停
	for i, v := range downhill_gravity_torque_arr {
		downhill_ht_upper_deviation_hover_arr = append(downhill_ht_upper_deviation_hover_arr, (ht_upper_deviation_spring_torque_arr[i]-v/2)/pole_force_arm_arr[i]/math.Cos(PI/180*vector_ab_xz_angle_arr[i])*1000)
	}

	// 下坡高温寿命后悬停
	for i, v := range downhill_gravity_torque_arr {
		downhill_ht_after_life_hover_arr = append(downhill_ht_after_life_hover_arr, (ht_after_life_spring_torque_arr[i]-v/2)/pole_force_arm_arr[i]/math.Cos(PI/180*vector_ab_xz_angle_arr[i])*1000)
	}

	// temp := []float64{200, 300}
	//
	// if cd_max_force > od_min_force {
	// 	return (flat_slope_nt_median_manually_open_door_arr + flat_slope_nt_median_manually_close_door_arr)
	// } else {
	// 	return temp
	// }
	var abcd []float64
	for i, v := range uphill_ht_lower_deviation_hover_arr {
		abcd = append(abcd, v, uphill_ht_median_hover_arr[i], uphill_ht_upper_deviation_hover_arr[i], uphill_ht_after_life_hover_arr[i], flat_slope_ht_lower_deviation_hover_arr[i], flat_slope_ht_median_hover_arr[i], flat_slope_ht_upper_deviation_hover_arr[i], flat_slope_ht_after_life_hover_arr[i], downhill_ht_lower_deviation_hover_arr[i], downhill_ht_median_hover_arr[i], downhill_ht_upper_deviation_hover_arr[i], downhill_ht_after_life_hover_arr[i])
	}

	min, max := ExtremumInArray(abcd)
	// var result int
	// if min >= (close_static_friction+50) && max <= (open_static_friction-50) {
	// 	result = 1
	// } else {
	// 	result = 0
	// }
	// return result
	return min, max
}

func main() {}

// 	bodax := []float64{3459.89107225708, 3460.39430290314, 3469.96843653133, 3465.0}
// 	boday := []float64{-582.946960168526, -582.101657439355, -581.313600104464, -575.25}
// 	bodaz := []float64{1110.52734375, 1120.67224121094, 1130.86755371094, 1134.5}
// 	for i, v := range bodax {
// 		fmt.Println(Formula_calculation(v, boday[i], bodaz[i]))
// 		time.Sleep(1 * time.Second)
// 	}
// }
