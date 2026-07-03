#version 460
#extension GL_GOOGLE_include_directive:enable

#include "cloud_common_noise.glsl"


#define kDetailFrequency 8.0//经验值，后期可调参
//============================
//体积云Worley细节噪声烘培
//============================

layout (set=0,binding=0,r8) uniform image3D imageWorleyNoise;
//Vulkan 不会按变量名 imageWorleyNoise自动找显存，只认(set, binding)这对编号,C++端也必须把对应VkImage绑到set 0，binding 0
//对 cloud_detailnoise  只有一个输出纹理的compute shader，set=0, binding=0 最简单：一个资源、一个插口
//uniform 是 GLSL 语法要求，表示这是一个 从外部绑进来的资源，由 C++ 在 draw/dispatch 前绑定
//和 cloud_basicnoise 那行对比:格式和绑定写法相同，因为都是单通道3D烘焙输出
//两个是 不同 compute shader，各自有独立的 set 0，互不冲突

layout (local_size_x=8,local_size_y=8,local_size_z=1) in;
//这是 Compute Shader（计算着色器）的工作组大小声明，告诉 GPU：每次派发的一个工作组里有多少个线程
//layout() 给 Vulkan的固定配置
//local_size_x X方向8个线程
//local_size_y Y方向8个线程
//local_size_z Z方向1个线程
//in 表示这是输入到着色器的布局

//local_size为(8,8,1)
//如果Cpp正确分配dispatch=(4,4,32),能正确画32*32*32大小的3D纹理，3D纹理中的每一个合法体素恰好有一个对应线程
//每个体素的对应全局线程ID是工作组ID*工作组大小+组内ID，即gl_GlobalInvocationID = gl_WorkGroupID * local_size + gl_LocalInvocationID
//体素坐标能与全局线程ID能设置成完全相同
//体素坐标(3, 5, 0) 则组ID (0,0,0)  组内ID (3,5,0) 全局线程ID(3, 5, 0)
//体素坐标(8, 0, 0) 则组ID (1,0,0)  组内ID (0,0,0) 全局线程ID(8, 0, 0)
//体素坐标(31, 31, 31) 则组ID (3,3,31) 组内ID (7,7,0) 全局线程ID(31,31, 31)


void main()
{
    ivec3 texSize=imageSize(imageWorleyNoise);
    ivec3 workPos=ivec3(gl_GlobalInvocationID.xyz);

    if(workPos.x >= texSize.x || workPos.y >= texSize.y || workPos.z >= texSize.z)
    {
        return;
    }//多派的线程不写越界体素

    const vec3 uvw=(vec3(workPos)+vec3(0.5))/vec3(texSize);
    //workPos：整数格点
    //+ 0.5：移到体素中心
    //除以texSize：归一化到 [0, 1]³
    //后面 worleyFbm(uvw, ...) 在这个连续坐标上采样。

    float detailNoise=worleyFbm(uvw,kDetailFrequency*1.0)*0.625+
    worleyFbm(uvw,kDetailFrequency*2.0)*0.25+
    worleyFbm(uvw,kDetailFrequency*4.0)*0.125;
    //三档Worley Fbm噪声加权混合，每一档 worleyFbm 内部已经叠了 3 层 Worley（0.625/0.25/0.125），这里再按 5:2:1 做 第二次 跨尺度混合
    //视觉效果：云边缘的 碎屑、镂空、毛边

    imageStore(imageWorleyNoise,workPos,vec4(detailNoise));//imageStore写入3D纹理
    //vec4类型被imageStore函数签名固定
    //这里实际上只写入了r分量，gba被忽略
}

//现状：
//算法完整，可编译为 compute shader
//未加入 compile_shaders.bat，C++ 未绑 compute + STORAGE_IMAGE
//输出 r8 单通道；运行时 cloud.frag 读 uDetailTex3D.a，对接前需统一通道/格式
//与 vk_scene.h CPU Detail 烘焙算法、频率、通道布局不一致
//将替换CPU烘培
