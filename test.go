package main

import (
	"fmt"
	"math"
)

type Vertex struct {
	vector_og_on_xz, vector_ob_on_xz float64
}
var vector_og_on_xz, vector_ob_on_xz, vector_ab, vector_ob_on_xz_x_angle, vector_og_on_xz_x_angle, o_centre_gravity_x, o_centre_gravity_z, o_gate_b1_x, o_gate_b1_z float64
var vector_ob1_length_x, vector_ob1_length_y, vector_ob1_length_z, vector_ab1_length_x, vector_ab1_length_y, vector_ab1_length_z, vector_ab_length_x, vector_ab_length_y, vector_ab_length_z, vector_ab1_ob1_on_xz_angle, od_length, cd_length, working_stroke, car_climbing_angle, vector_ob_slope, vector_ab_slope, vector_ob_ab_angle float64
var vector_ab_xz_angle, vector_ab1_xz_angle, od_pole_force_arm, cd_pole_force_arm, od_min_force, cd_max_force, spring_cd_length, spring_od_length, deformation, max_test_shear_stress, dynamic_load_allowable_shear_stress, initial_mean_diameter, spring_theoretical_rate float64

const PI = 3.141592653589793
const Initial_wire_diameter = 3.6 // 弹簧初选线径

func Formula_calculation(a_x, a_y, a_z float64) float64 {
// 	var vector_og_on_xz, vector_ob_on_xz, vector_ab, vector_ob_on_xz_x_angle, vector_og_on_xz_x_angle, o_centre_gravity_x, o_centre_gravity_z, o_gate_b1_x, o_gate_b1_z float64
// 	var vector_ob1_length_x, vector_ob1_length_y, vector_ob1_length_z, vector_ab1_length_x, vector_ab1_length_y, vector_ab1_length_z float64
 	centre_gravity_x := 3850.0
 	centre_gravity_z := 853.576
 	hinge_x := 3470.5
 	hinge_y := 0.0
 	hinge_z := 1240.0
 	open_angle := 89.0
	door_weight := 32.31
	climbing_degree := 0.2
 	b_x := 3711.0
 	b_y := -659.67
 	b_z := 745.25
	
	platform_spring_static_length := 224.1
	platform_max_tensile_strength := 2100.0
	platform_inside_diameter := 21.55
	

	vector_og_on_xz = math.Sqrt((centre_gravity_x-hinge_x)*(centre_gravity_x-hinge_x) + (centre_gravity_z-hinge_z)*(centre_gravity_z-hinge_z))
	// 向量OG在XZ平面的投影长度
	vector_ob_on_xz = math.Sqrt(math.Pow((b_x-hinge_x),2) + math.Pow((b_z-hinge_z),2))
	// 向量OB在XZ平面的投影长度
	vector_ab = math.Sqrt((a_x-b_x)*(a_x-b_x) + (a_y-b_y)*(a_y-b_y) + (a_z-b_z)*(a_z-b_z))
	// 向量AB的长度,即撑杆关门长度
	vector_ob_on_xz_x_angle = (180 / PI) * math.Atan((b_z-hinge_z)/(b_x-hinge_x))
	// 向量OB在XZ平面与X轴的夹角（°）
	vector_og_on_xz_x_angle = (180 / PI) * math.Atan((centre_gravity_z-hinge_z)/(centre_gravity_x-hinge_x))
	// 向量OG在XZ平面与X轴的夹角（°）
	o_centre_gravity_x = vector_og_on_xz*math.Cos((open_angle-math.Abs(vector_og_on_xz_x_angle))*PI/180) + hinge_x
	// 开门重心X坐标
	o_centre_gravity_z = vector_og_on_xz*math.Sin((open_angle-math.Abs(vector_og_on_xz_x_angle))*PI/180) + hinge_z
	// 开门重心z坐标

	o_gate_b1_x = vector_ob_on_xz*math.Cos((open_angle-math.Abs(vector_ob_on_xz_x_angle))*PI/180) + hinge_x
	// 开门尾门连接点B1 X坐标
	o_gate_b1_z = vector_ob_on_xz*math.Sin((open_angle-math.Abs(vector_ob_on_xz_x_angle))*PI/180) + hinge_z
	// 开门尾门连接点B1 z坐标
	vector_ob1_length_x = o_gate_b1_x - hinge_x
	// 向量OB1的长度（X方向分量）

	vector_ob1_length_y = math.Abs(b_y - hinge_y)
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
	
	vector_ab1_ob1_on_xz_angle = (180/PI) * math.Acos((vector_ob1_length_x * vector_ab1_length_x + vector_ob1_length_z * vector_ab1_length_z) / (math.Sqrt(vector_ob1_length_x * vector_ob1_length_x + vector_ob1_length_z * vector_ob1_length_z) * math.Sqrt(vector_ab1_length_x * vector_ab1_length_x + vector_ab1_length_z * vector_ab1_length_z)))
	// 向量AB1和OB1在XZ平面的夹角（°）
	od_length = math.Sqrt(math.Pow((o_gate_b1_x - a_x),2) + math.Pow((b_y - a_y) ,2) + math.Pow((o_gate_b1_z - a_z) ,2))
	// 撑杆开门长度
	cd_length = math.Sqrt(math.Pow((b_x - a_x),2) + math.Pow((b_y - a_y), 2) + math.Pow((b_z - a_z), 2))
	// 撑杆关门长度
	working_stroke = od_length - cd_length // 撑杆工作行程
	car_climbing_angle = math.Atan(climbing_degree ) * 180 / PI  // 汽车爬坡角度
	
	vector_ob_slope = (b_z - hinge_z) / (b_x - hinge_x) // 向量ob的斜率
	vector_ab_slope = (b_z - a_z) / (b_x - a_x) // 向量ab的斜率
	vector_ob_ab_angle = (180/PI) *  math.Atan((vector_ab_slope - vector_ob_slope) / (1 + vector_ab_slope * vector_ob_slope))
	// 向量OB和向量AB的夹角
	
	vector_ab_xz_angle = math.Acos(math.Sqrt(math.Pow(vector_ab_length_x, 2) + math.Pow(vector_ab_length_z, 2)) / math.Sqrt(math.Pow(vector_ab_length_x, 2) + math.Pow(vector_ab_length_y, 2) + math.Pow(vector_ab_length_z, 2)) ) * 180 / PI
	// 向量AB与XZ平面的线面夹角（空间夹角）
	vector_ab1_xz_angle = math.Acos(math.Sqrt(math.Pow(vector_ab1_length_x, 2) + math.Pow(vector_ab1_length_z, 2)) / math.Sqrt(math.Pow(vector_ab1_length_x, 2) + math.Pow(vector_ab1_length_y, 2) + math.Pow(vector_ab1_length_z, 2)) ) * 180 / PI
	// 向量AB1与XZ平面的线面夹角（空间夹角）
	
	od_pole_force_arm = vector_ob_on_xz * math.Sin(vector_ab1_ob1_on_xz_angle * PI / 180)
	// 开门撑杆力臂
	cd_pole_force_arm = vector_ob_on_xz * math.Sin(vector_ob_ab_angle * PI / 180) 
	// 关门撑杆力臂
	od_min_force = door_weight * 9.8 * (o_centre_gravity_x - hinge_x ) / (2 * od_pole_force_arm) / math.Cos(vector_ab1_xz_angle * PI / 180 )
	// 开门在最高点需求力
	cd_max_force = door_weight * 9.8 * (centre_gravity_x - hinge_x ) / (2 * cd_pole_force_arm) / math.Cos(vector_ab_xz_angle * PI / 180 )
	// 关门到开门瞬间时需求力
 
	// 弹簧计算 //
	
	spring_cd_length = cd_length - platform_spring_static_length
	spring_od_length = od_length - platform_spring_static_length
	deformation = spring_od_length - spring_cd_length
	max_test_shear_stress = platform_max_tensile_strength * 0.6  // 最大试验切应用力
	dynamic_load_allowable_shear_stress = platform_max_tensile_strength * 0.57 // 动负荷许用切应力
	initial_mean_diameter = platform_inside_diameter + Initial_wire_diameter // 弹簧初选中径
	spring_theoretical_rate = (cd_max_force - od_min_force) / deformation // 理论刚度
	
	return spring_theoretical_rate
}

func main() {
	var abc float64
	abc = Formula_calculation(3465.0, -575.0, 1134.0)
	fmt.Printf("%v\n", abc)
	// fmt.Printf("vector_ob_on_xz_x_angle= %v vector_ab= %v \n",
	// 	vector_og_on_xz, vector_ab)
}
