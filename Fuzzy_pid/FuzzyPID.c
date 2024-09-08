#include "FuzzyPID.h"
#define NB -3
#define NM - 2
#define NS - 1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3
int  Kp_rule[7][7] = { {PB,PB,PM,PM,PS,ZO,ZO},     //kp规则表
                               {PB,PB,PM,PS,PS,ZO,NS},
                               {PM,PM,PM,PS,ZO,NS,NS},
                               {PM,PM,PS,ZO,NS,NM,NM},
                               {PS,PS,ZO,NS,NS,NM,NM},
                               {PS,ZO,NS,NM,NM,NM,NB},
                               {ZO,ZO,NM,NM,NM,NB,NB} };

int  Ki_rule[7][7] = { {NB,NB,NM,NM,NS,ZO,ZO},     //ki规则表
                            {NB,NB,NM,NS,NS,ZO,ZO},
                            {NB,NM,NS,NS,ZO,PS,PS},
                            {NM,NM,NS,ZO,PS,PM,PM},
                            {NM,NS,ZO,PS,PS,PM,PB},
                            {ZO,ZO,PS,PS,PM,PB,PB},
                            {ZO,ZO,PS,PM,PM,PB,PB} };

int  Kd_rule[7][7] = { {PS,NS,NB,NB,NB,NM,PS},    //kd规则表
                            {PS,NS,NB,NM,NM,NS,ZO},
                            {ZO,NS,NM,NM,NS,NS,ZO},
                            {ZO,NS,NS,NS,NS,NS,ZO},
                            {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
                            {PB,NS,PS,PS,PS,PS,PB},
                            {PB,PM,PM,PM,PS,PS,PB} };

int  Fuzzy_rule[7][7] = { {PB,PB,PB,PB,PM,ZO,ZO},
                               {PB,PB,PB,PM,PM,ZO,ZO},
                               {PB,PM,PM,PS,ZO,NS,NM},
                               {PM,PM,PS,ZO,NS,NM,NM},
                               {PS,PS,ZO,NM,NM,NM,NB},
                               {ZO,ZO,ZO,NM,NB,NB,NB},
                               {ZO,NS,NB,NB,NB,NB,NB} };
float values[7] = { -3,-2,-1,0,1,2,3 }; //输入e的隶属值

typedef struct FuzzyPID
{
       int  num_area ; //划分区域个数
    //float e_max;  //误差最大值
    //float e_min;  //误差最小值
    //float ec_max;  //误差变化最大值
    //float ec_min;  //误差变化最小值
    //float kp_max, kp_min;
    float e_membership_values[7] ; //输入e的隶属值（只能是-3、-2...3)
    float ec_membership_values[7] ;//输入de/dt的隶属值（只能是-3、-2...3)
    float kp_menbership_values[7] ;//输出增量kp的隶属值（只能是-3、-2...3)
    float ki_menbership_values[7] ; //输出增量ki的隶属值（只能是-3、-2...3)
    float kd_menbership_values[7] ;  //输出增量kd的隶属值（只能是-3、-2...3)
    float fuzzyoutput_menbership_values[7];

    //int menbership_values[7] = {-3,-};
    float kp;                       //PID参数kp
    float ki;                       //PID参数ki
    float kd;                       //PID参数kd
    float qdetail_kp;               //增量kp对应论域中的值（未完全解模糊化，在[-3,3]之间的数值，下一步就是量化）
    float qdetail_ki;               //增量ki对应论域中的值（未完全解模糊化，在[-3,3]之间的数值，下一步就是量化）
    float qdetail_kd;               //增量kd对应论域中的值（未完全解模糊化，在[-3,3]之间的数值，下一步就是量化）
    float qfuzzy_output;
    float detail_kp;                //输出增量kp（完全解模糊化（量化后的）以后具体的数值）
    float detail_ki;                //输出增量ki（完全解模糊化（量化后的）以后具体的数值）
    float detail_kd;                //输出增量kd（完全解模糊化（量化后的）以后具体的数值）
    float fuzzy_output;
    float qerro;                    //输入e对应论域中的值（量化处理后在[-3,3]之间的数值）
    float qerro_c;                  //输入de/dt对应论域中的值（量化处理后在[-3,3]之间的数值）
    float errosum;
    float e_gradmembership[2];      //输入e的隶属度(对应两个模糊量的隶属度，是[-3,3]之间的值)
    float ec_gradmembership[2];     //输入de/dt的隶属度((对应两个模糊量的隶属度，是[-3,3]之间的值)
    int e_grad_index[2];            //输入e隶属度在规则表的索引（可以指示着对应哪两个模糊量）
    int ec_grad_index[2];           //输入de/dt隶属度在规则表的索引（可以指示着对应哪两个模糊量）
    float gradSums[7] ;
    float KpgradSums[7];   //输出增量kp总的隶属度
    float KigradSums[7] ;   //输出增量ki总的隶属度
    float KdgradSums[7] ;   //输出增量kd总的隶属度
   

    int  Kp_rule_list[7][7];

    int  Ki_rule_list[7][7];

    int  Kd_rule_list[7][7];

    int  Fuzzy_rule_list[7][7];


    //private:

}FuzzyPID;

void FuzzyPID_Init(FuzzyPID* pid)  //构造函数，配置模糊PID控制器的结构体 FuzzyPID。
{
    int i, j;
    pid->num_area = 8;//区域个数为8
    pid->kp = 0;//pid参数kp为0
    pid->ki = 0;//pid参数ki为0
    pid->kd = 0;//pid参数kd为0
    pid->fuzzy_output = 0;//模糊控制器输出初始值，设置为 0。
    pid->qdetail_kp = 0;//增量kp对应论域中的值为0
    pid->qdetail_ki = 0;//增量ki对应论域中的值为0
    pid->qdetail_kd = 0;//增量kd对应论域中的值为0
    pid->qfuzzy_output = 0;
    pid->errosum = 0;
    for ( i = 0; i < 7; i++)//通过循环初始化模糊控制表
    {
        for ( j = 0; j < 7; j++)
        {
            pid->Kp_rule_list[i][j] = Kp_rule[i][j];
            pid->Ki_rule_list[i][j] = Ki_rule[i][j];
            pid->Kd_rule_list[i][j] = Kd_rule[i][j];
            pid->Fuzzy_rule_list[i][j] = Fuzzy_rule[i][j];
        }
    }
    for ( i = 0; i < 7; i++)
    {
        pid->e_membership_values[i] = values[i];//输入e的隶属值初始化，使其对应values数组
        pid->ec_membership_values[i] = values[i];//输入de/dt的隶属值初始化，使其对应values数组
        pid->kp_menbership_values[i] = values[i];//输出增量kp的隶属值初始化，使其对应values数组
        pid->ki_menbership_values[i] = values[i];//输出增量ki的隶属值初始化，使其对应values数组
        pid->kd_menbership_values[i] = values[i];//输出增量kd的隶属值初始化，使其对应values数组
        pid->fuzzyoutput_menbership_values[i] = values[i];
        pid->gradSums[i] = 0;
        pid->KpgradSums[i] = 0;//输出增量kp总的隶属度初始化为0
        pid->KigradSums[i] = 0;//输出增量ki总的隶属度初始化为0
        pid->KdgradSums[i] = 0;//输出增量kd总的隶属度初始化为0
    }
}




//输入e与de/dt隶属度计算函数///
void Get_grad_membership(FuzzyPID* pid,float erro, float erro_c)//计算误差（erro）和误差的微分（erro_c）的隶属度。接受三个参数：指向FuzzyPID结构体的指针pid、误差erro和误差变化率erro_c。
{
    int i;//声明一个整型变量i，用于循环控制。
    //当误差在这个范围
    if (erro > pid->e_membership_values[0] && erro < pid->e_membership_values[6])//如果误差erro在e_membership_values数组定义的论域范围内，则
    {

        //6个区域
        for ( i = 0; i < pid->num_area - 2; i++)//遍历6个的模糊区域
        {
            //如果误差在区间区域内
            if (erro >= pid->e_membership_values[i] && erro <= pid->e_membership_values[i + 1])//如果erro在e_membership_values[]第i和i+1之间
            {
                //e的隶属度，一个e对应两个模糊集都有隶属度的值
                //对于PM正中模糊集合的隶属度
                pid->e_gradmembership[0] = -(erro - pid->e_membership_values[i + 1]) / (pid->e_membership_values[i + 1] - pid->e_membership_values[i]);//e的隶属度数组的第一个变量给PM对应的隶属度
                //PB
                pid->e_gradmembership[1] = 1 + (erro - pid->e_membership_values[i + 1]) / (pid->e_membership_values[i + 1] - pid->e_membership_values[i]);//e的隶属度数组的第二个变量给PM对应的隶属度
                //记录是在哪两个区间内
                pid->e_grad_index[0] = i;//输入e隶属度在规则表的索引数组第一个变量为i（决定了e的隶属度属于哪个模糊集合）
                pid->e_grad_index[1] = i + 1;//输入e隶属度在规则表的索引数组第二个变量为i+1
                break;
            }
        }
    }
    else//如果误差erro不在e_membership_values数组定义的范围内，（处理误差在模糊区域范围外的情况。）则
    {
        //如果误差的止小于等于论域的最小值
        if (erro <= pid->e_membership_values[0])//如果erro小于等于论域的最小值
        {
            pid->e_gradmembership[0] = 1;//e的隶属度数组第一个变量为1
            pid->e_gradmembership[1] = 0;//e的隶属度数组第一个变量为0
            pid->e_grad_index[0] = 0;//将e_grad_index数组的第一个元素设置为 0，指示论域最小区域的索引。
            pid->e_grad_index[1] = -1;//将e_grad_index数组的第二个元素设置为 -1，表示没有后续区域（超出定义范围）。
        }//超出范围了
        else if (erro >= pid->e_membership_values[6])如果erro大于等于论域的最大值
        {
            pid->e_gradmembership[0] = 1;//e的隶属度数组第一个变量为1
            pid->e_gradmembership[1] = 0;//e的隶属度数组第一个变量为0
            pid->e_grad_index[0] = 6;//将e_grad_index数组的第一个元素设置为 0，指示论域最大区域的索引。
            pid->e_grad_index[1] = -1;//将e_grad_index数组的第二个元素设置为 -1，表示没有后续区域（超出定义范围）。
        }
    }
    //误差的微分
    if (erro_c > pid->ec_membership_values[0] && erro_c < pid->ec_membership_values[6])//如果误差微分erro_c在ec_membership_values数组定义的论域范围内，则
    {
        for ( i = 0; i < pid->num_area - 2; i++)//遍历6个的模糊区域
        {
            if (erro_c >= pid->ec_membership_values[i] && erro_c <= pid->ec_membership_values[i + 1])////如果erro_c在ec_membership_values[]第i和i+1之间
            {
                pid->ec_gradmembership[0] = -(erro_c - pid->ec_membership_values[i + 1]) / (pid->ec_membership_values[i + 1] - pid->ec_membership_values[i]);
                pid->ec_gradmembership[1] = 1 + (erro_c - pid->ec_membership_values[i + 1]) / (pid->ec_membership_values[i + 1] - pid->ec_membership_values[i]);
                pid->ec_grad_index[0] = i;
                pid->ec_grad_index[1] = i + 1;
                break;
            }
        }
    }
    else
    {
        if (erro_c <= pid->ec_membership_values[0])
        {
            pid->ec_gradmembership[0] = 1;
            pid->ec_gradmembership[1] = 0;
            pid->ec_grad_index[0] = 0;
            pid->ec_grad_index[1] = -1;
        }
        else if (erro_c >= pid->ec_membership_values[6])
        {
            pid->ec_gradmembership[0] = 1;
            pid->ec_gradmembership[1] = 0;
            pid->ec_grad_index[0] = 6;
            pid->ec_grad_index[1] = -1;
        }
    }

}

// //获取输出增量kp, ki, kd的总隶属度 /
void GetSumGrad(FuzzyPID* pid)//计算模糊PID控制器中增量 Kp, Ki, 和 Kd 的总隶属度。
{
    int i,j;//声明两个整型变量 i 和 j，用于循环迭代。
    //划分八个区域
    for ( i = 0; i <= pid->num_area - 1; i++)//执行7次循环
    {
        pid->KpgradSums[i] = 0;//输出增量kp总的隶属度清0
        pid->KigradSums[i] = 0;//输出增量ki总的隶属度清0
        pid->KdgradSums[i] = 0;//输出增量kd总的隶属度清0
        //把PID的各个隶属值清零
    }
    for ( i = 0; i < 2; i++)//循环两次
    {
        if (pid->e_grad_index[i] == -1)//误差爆表，如果e_grad_indexe隶属度在规则表的索引的第i个元素为-1
          //通过检查 pid->e_grad_index[i] 和 pid->ec_grad_index[j] 是否为-1，判断当前误差和误差变化率是否有效。如果某个索引为-1，说明该状态的误差超出了定义范围，因此跳过该计算。
        {
            continue;//如果在 continue 语句后面还有其他代码，continue 语句会跳过这些代码，直接进入下一次for迭代。
        }
        for ( j = 0; j < 2; j++)//如果误差e_grad_indexe隶属度在规则表的索引的第i个元素不是-1，循环两次
        {
            if (pid->ec_grad_index[j] != -1)//误差的微分有没有爆表，如果误差的微分ec_grad_index隶属度在规则表的索引的第j个元素不是-1，则
            {
                int indexKp = pid->Kp_rule_list[pid->e_grad_index[i]][pid->ec_grad_index[j]] + 3;//输出增量kp总的隶属度数组的指数=kp规则表对应的值+3；就变成了0~6，7个数可以作为增量kp总的隶属度数组的指数
                int indexKi = pid->Ki_rule_list[pid->e_grad_index[i]][pid->ec_grad_index[j]] + 3;//输出增量ki总的隶属度数组的指数=ki规则表对应的值+3；就变成了0~6，7个数可以作为增量ki总的隶属度数组的指数
                int indexKd = pid->Kd_rule_list[pid->e_grad_index[i]][pid->ec_grad_index[j]] + 3;//输出增量kd总的隶属度数组的指数=kd规则表对应的值+3；就变成了0~6，7个数可以作为增量kd总的隶属度数组的指数
                //gradSums[index] = gradSums[index] + (e_gradmembership[i] * ec_gradmembership[j])* Kp_rule_list[e_grad_index[i]][ec_grad_index[j]];
                pid->KpgradSums[indexKp] = pid->KpgradSums[indexKp] + (pid->e_gradmembership[i] * pid->ec_gradmembership[j]);//输出增量kp总的隶属度=输出增量kp上一次总的隶属度+误差对应隶属度的数值*误差变量对应隶属度的数值
                pid->KigradSums[indexKi] = pid->KigradSums[indexKi] + (pid->e_gradmembership[i] * pid->ec_gradmembership[j]);
                pid->KdgradSums[indexKd] = pid->KdgradSums[indexKd] + (pid->e_gradmembership[i] * pid->ec_gradmembership[j]);
            }
            else//否则，误差的微分ec_grad_index隶属度在规则表的索引的第j个元素是-1，进入下一次迭代
            {
                continue;
            }

        }
    }

}

//计算输出增量kp, kd, ki对应论域值//
void GetOUT(FuzzyPID* pid)//计算并更新三个增量kp, ki, 和 kd 对应的论域值。
{
    int i;
    for ( i = 0; i < pid->num_area - 1; i++)//执行7次循环
    {
        pid->qdetail_kp +=pid->kp_menbership_values[i] * pid->KpgradSums[i];//增量kp对应论域中的值=增量kp对应论域中的值+输出增量kd的隶属值（只能是[-3,3]）*输出增量kp总的隶属度
        pid->qdetail_ki += pid->ki_menbership_values[i] * pid->KigradSums[i];
        pid->qdetail_kd += pid->kd_menbership_values[i] * pid->KdgradSums[i];
    }
}

//模糊PID控制实现函数/
float FuzzyPIDcontroller(FuzzyPID* pid, range* rang, Error* error, float Target, float actual)//模糊PID控制器的函数，计算控制输出。接受FuzzyPID、range、Error 结构体的指针，以及目标值Target 和实际值actual。
{
    
    error->erro_ppre = error->erro_pre;//将上一次误差值赋给上上次误差值。
    error->erro_pre = error->erro;//将当前误差值赋给上一次误差值。
    error->erro = Target - actual;//计算当前误差，即目标值与实际值之差。
    error->erro_c = error->erro - error->erro_pre;//计算误差的微分，即当前误差与上一次误差之差。
    pid->errosum += error->erro;//将当前误差值累加到误差和中。
    //Arear_dipart(e_max, e_min, ec_max, ec_min, kp_max, kp_min,ki_max,ki_min,kd_max,kd_min);
    pid->qerro = Quantization(rang->e_max, rang->e_min, error->erro);//区间映射，对当前误差值进行量化处理，将其映射到指定范围。
    pid->qerro_c = Quantization(rang->ec_max, rang->ec_min, error->erro_c);//区间映射，对误差微分进行量化处理。
    //把他们缩小到0123范围内
    Get_grad_membership(pid,pid->qerro, pid->qerro_c);//根据量化后的误差和误差微分计算模糊隶属度。
    //获取输出增量kp, ki, kd的总隶属度
    GetSumGrad(pid);//计算隶属度的总和，用于决定PID参数的调整。
    //计算输出增量kp, kd, ki对应论域值//
    GetOUT(pid);//计算PID增量的输出。
    pid->detail_kp = Inverse_quantization(rang->kp_max, rang->kp_min, pid->qdetail_kp);//对量化后的PID增量qdetail_kp（kp对应论域中的值）进行反量化，得到实际的增量值。
    pid->detail_ki = Inverse_quantization(rang->ki_max, rang->ki_min, pid->qdetail_ki);//对量化后的PID增量qdetail_kp（ki对应论域中的值）进行反量化，得到实际的增量值。
    pid->detail_kd = Inverse_quantization(rang->kd_max, rang->kd_min, pid->qdetail_kd);//对量化后的PID增量qdetail_kp（kd对应论域中的值）进行反量化，得到实际的增量值。
    pid->qdetail_kd = 0;//将量化增量设置为0。
    pid->qdetail_ki = 0;//将量化增量设置为0
    pid->qdetail_kp = 0;//将量化增量设置为0
    /*if (qdetail_kp >= kp_max)
        qdetail_kp = kp_max;
    else if (qdetail_kp <= kp_min)
        qdetail_kp = kp_min;
    if (qdetail_ki >= ki_max)
        qdetail_ki = ki_max;
    else if (qdetail_ki <= ki_min)
        qdetail_ki = ki_min;
    if (qdetail_kd >= kd_max)
        qdetail_kd = kd_max;
    else if (qdetail_kd <= kd_min)
        qdetail_kd = kd_min;*/
    pid->kp = pid->kp + pid->detail_kp;//更新 kp 的值。
    pid->ki = pid->ki + pid->detail_ki;//更新 ki 的值。
    pid->kd  =pid->kd + pid->detail_kd;//更新 kd 的值。
    //确定范围
    if (pid->kp < 0)//确保 kp 不小于0
        pid->kp = 0;
    if (pid->ki < 0)//确保 ki 不小于0。
        pid->ki = 0;
    if (pid->kd < 0)//确保 kd 不小于0。
        pid->kd = 0;
    pid->detail_kp = 0;//将增量重置为0
    pid->detail_ki = 0;
    pid->detail_kd = 0;
    //增量式PID
    float output = pid->kp * (error->erro - error->erro_pre) + pid->ki * error->erro + pid->kd * (error->erro - 2 * error->erro_pre + error->erro_ppre);//根据增量式PID公式计算输出值。
    return output;//返回计算得到的控制输出。
}

///区间映射函数///
float Quantization(float maximum, float minimum, float x)//将 x 映射到一个新的区间[-3,3].它接收三个参数：maximum（区间的最大值），minimum（区间的最小值），和 x（待量化的数值），并返回一个 float 类型的结果。
{
    float qvalues = 6.0 * (x - minimum) / (maximum - minimum) - 3;
    //float qvalues=6.0*()
    return qvalues;

    //qvalues[1] = 3.0 * ecerro / (maximum - minimum);
}

//反区间映射函数
float Inverse_quantization(float maximum, float minimum, float qvalues)//将量化值qvalues从 [-3, 3]区间映射回 [minimum, maximum] 区间
{
    float x = (maximum - minimum) * (qvalues + 3) / 6 + minimum;
    return x;
}
