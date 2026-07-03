#version 460
#extension GL_GOOGLE_include_directive : enable
//GLSL版本
//启用Goohle的include扩展，允许你#include文件

#include "cloud_common_noise.glsl"

#define kBasicFrequency 4.0
#define kBasicNoiseMixFactor 0.5

//============================
//Perlin-Worley 基础噪声烘培
//============================


//体积云一般分两步
//【烘焙阶段】Compute Shader 算噪声 → 写入 3D 纹理
                    ↓
//【运行阶段】Fragment Shader 采样 3D 纹理 → 画云


layout (set=0,binding=0,r8) uniform image3D imageBasicNoise;
//set 描述符集
//layout(...)vulkan描述符布局，给Vulkan看的标注，该变量在第几个set，在这个set的第几个接口
//binding=0,该set第0号接口
//uniform image3D 3D图像变量，可读可写
//imageBasicNoise 变量名，C++ 侧绑同名的 VkImage
//r8 R8_UNORM 一类单通道8位格式，告诉Vulkan这个image3D,每个体素用多少位，存几个通道，C++ 创建对应 VkImage 时格式要一致，用VK_FORMAT_R8_UNORM
//这是 Vulkan Compute Shader 里的 3D 图像存储绑定：
//声明一块可由着色器直接读写的 3D 纹理，用来烘焙基础噪声 Perlin-Worley 到 imageBasicNoise

float remap(float x,float a,float b,float c,float d)
{
    return (((x-a)/(b-a))*(d-c))+c;
}//线性范围重映射，按x在[a,b]的比例取在[c,d]中的对应值

float basicNoiseComposite(vec4 v)//vec4类型的v装了4层噪声，v.x装PerlinFBM，v.y装Worley第一层，v.z装Worley第二层，v.w装Worley第三层
{
    float wfbm=v.y*0.625+v.z*0.25+v.w*0.125;//3层Worley加权混合

    //cloud shape modeled after the GPU Pro 7 chapter
    return remap(v.x,wfbm-1.0,1.0,0.0,1.0);//remap 把 Perlin 从区间 [wfbm-1, 1] 映射到 [0, 1],wfbm越大，侵蚀越强，有云的下限越高，镂空更多
}

layout(local_size_x=8,local_size_y=8,local_size_z=1) in;
//这是 Compute Shader（计算着色器）的工作组大小声明，告诉 GPU：每次派发的一个工作组里有多少个线程
//layout() 给 Vulkan的固定配置
//local_size_x X方向8个线程
//local_size_y Y方向8个线程
//local_size_z Z方向1个线程
//in 表示这是输入到着色器的布局

//local_size为(8,8,1)
//如果Cpp正确分配dispatch=(16,16,128),能正确画128*128*128大小的3D纹理，3D纹理中的每一个合法体素恰好有一个对应线程
//每个体素的对应全局线程ID是工作组ID*工作组大小+组内ID，即gl_GlobalInvocationID = gl_WorkGroupID * local_size + gl_LocalInvocationID
//体素坐标能与全局线程ID能设置成完全相同
//体素坐标(3, 5, 0) 则组ID (0,0,0)  组内ID (3,5,0) 全局线程ID(3, 5, 0)
//体素坐标(8, 0, 0) 则组ID (1,0,0)  组内ID (0,0,0) 全局线程ID(8, 0, 0)
//体素坐标(127, 127, 127) 则组ID (15,15,127) 组内ID (7,7,0) 全局线程ID(127, 127, 127)


void main()
{
    ivec3 texSize=imageSize(imageBasicNoise);// 纹理多大
    ivec3 workPos=ivec3(gl_GlobalInvocationID.xyz);//当前线程负责的体素索引。Compute shader 没有顶点，靠线程网格跑。每个线程用全局ID区分自己,gl_GlobalInvocationID全局线程坐标 = 组ID × 组大小 + 组内ID
    //体素，3D 纹理里的一个小立方体格子，存一个噪声值；
    //workPos 当前线程负责的那个体素的 (x, y, z)

    if(workPos.x>=texSize.x || workPos.y>=texSize.y ||workPos.z>=texSize.z)
    {
        return;
    }//如果线程编号(体素坐标)超出纹理范围，直接 return，不算噪声、不写入
    //GPU 只能按整组（8 个线程）派，不能派「半个组」
    //没有 if 时，多余线程会 imageStore 到非法坐标 → 未定义行为 / 崩溃 / 花屏

    const vec3 uvw=(vec3(workPos)+vec3(0.5))/vec3(texSize);
    //把整数格点移到体素中心，再归一化到[0, 1]³,vec3(workPos)+vec3(0.5)才是体素中心，一个体素边长为1

    float pfbm =mix(1.0,perlinFbm(uvw,kBasicFrequency,7),kBasicNoiseMixFactor);//7层较丰富，kBasicFrequency是平铺周期，mix(x,y,0.5)=(x+y)/2,避免噪声对比过强。
    pfbm=abs(pfbm*2.0-1.0);// billowy perlin noise，让云更加蓬松
    
    vec4 col=vec4(0.0);
    col.g+=worleyFbm(uvw,kBasicFrequency*1.0);
    col.b+=worleyFbm(uvw,kBasicFrequency*2.0);
    col.a+=worleyFbm(uvw,kBasicFrequency*4.0);
    //每一档 worleyFbm 内部已经是3层 Worley 叠加（0.625/0.25/0.125）；g/b/a 再交给 basicNoiseComposite 做第二次 5:2:1 混合。

    col.r+=remap(pfbm,0.0,1.0,col.g,1.0);// perlin-worley用粗 Worley 当「下限」，把 Perlin 从 [0,1] 压到 [col.g, 1],先粗侵蚀一部分Perlin
    //此时col.r是已和粗 Worley 混合过的 Perlin,col.g是粗 Worley FBM,col.b是中Worley FBM,col.a是细 Worley FBM


    imageStore(imageBasicNoise,workPos,vec4(basicNoiseComposite(col)));
    //basicNoiseComposite(col) 再做第二次混合,wfbm = col.g×0.625 + col.b×0.25 + col.a×0.125,返回remap(col.r, wfbm-1, 1, 0, 1)
    //结果写入workPos体素的r8单通道
}   

//现状：
//算法与vk_scene.h中的cpu烘培逻辑并不一致，它没有两阶段remap+basicNoiseComposite
//现在Cpp尚未绑定
//噪声函数可能需要放到cloud_common_noise.glsl中，该文件最好只保留layout和main函
//将替换CPU烘培