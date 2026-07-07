#version 460
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable

#include "cloud_common.glsl"

layout (local_size_x = 8, local_size_y = 8) in;

void main()
{//——————————————————————做时域重投影准备，把「当前全分辨率像素」对应的云，映射到上一帧屏幕上的哪个UV，从而判断能不能用history做TAA稳定——————————————————————————————————————
    ivec2 texSize = imageSize(imageCloudReconstructionTexture);
    ivec2 workPos = ivec2(gl_GlobalInvocationID.xy);//全分辨率像素坐标

    if(workPos.x >= texSize.x || workPos.y >= texSize.y)
    {
        return;
    }

    const vec2 uv = (vec2(workPos) + vec2(0.5f)) / vec2(texSize);

    const float traceCloudDepth = texelFetch(inCloudDepthTexture, workPos / 4, 0).r;//workPos/4.0对应1/4分辨率像素坐标，从inCloudDepthTexture中取样，得到该像素的云代表深度
    //texelFetch是GLSL内置函数，用于精确读取texel的对应值。与texture()不同，后者采样坐标为uv坐标，可以选择各种插值采样
    //texel是纹素,1920*1080大小的纹理有1920*1080个纹素，坐标为整数，而UV坐标为浮点[0,1],可以落在格与格之间
    
    const vec2 curEvaluateCloudTexelSize = 1.0f / vec2(textureSize(inCloudRenderTexture, 0));//先算好在1/4RT下一个texel在UV空间中的大小

    // Reproject to get prev uv.
    vec3 worlPosCur = getWorldPos(uv, traceCloudDepth, frameData);//getWorldPos来自shared_functions.glsl,输入全分辨率uv坐标和云深度，输出该像素云上一点的当前帧世界坐标
    vec4 projPosPrev = frameData.camViewProjPrev * vec4(worlPosCur, 1.0);//用 上一帧 camViewProjPrev 投到上一帧裁剪空间，再透视除法
    vec3 projPosPrevH = projPosPrev.xyz / projPosPrev.w;//得到上一帧屏幕位置

    vec2 uvPrev = projPosPrevH.xy * 0.5 + 0.5;//从裁剪空间NDC坐标[-1,1]转换成UV坐标[0,1]
    uvPrev.y = 1.0 - uvPrev.y;//翻转y坐标，裁剪空间 Y 向上，纹理 V 向下
    //uvPrev是上一帧history纹理的采样坐标

    bool bCameraCut = frameData.bCameraCut != 0;//切镜头改成ture

    // Valid check.
    const bool bPrevUvValid = onRange(uvPrev, vec2(0.0), vec2(1.0)) && (!bCameraCut);//重投影点还在屏幕内；飞出屏幕的历史无效；相机瞬切（切镜头/传送）时上一帧对不上，禁用 history
    //如果为true用history采样，为false只用当前 1/4 双线性上采样，不做时域混合

    //———————————————————————————完整时域重投影逻辑，在全分辨率上，把 1/4 分辨率的云数据「填洞 + 时域稳定」，输出三张全分辨率 RT————————————————————————————————————
    vec4 color = vec4(0.0);
    vec4 fog = vec4(0.0);
    float depthZ = 0.0;
    if(bPrevUvValid)//
    {
        // Evaluate, fetch it.
        vec4 curColor   = texelFetch(inCloudRenderTexture, workPos / 4, 0);
        vec4 curFog   = texelFetch(inCloudFogRenderTexture, workPos / 4, 0);
        float curDepthZ = texelFetch(inCloudDepthTexture,  workPos / 4, 0).r;

        float preDepthZ = texture(sampler2D(inCloudDepthReconstructionTextureHistory,  linearClampEdgeSampler), uvPrev).r;

        // Evaluate state check.
        uint  bayerIndex  = frameData.frameIndex.x % 16;//16帧一循环
        ivec2 bayerOffset = ivec2(kBayerMatrix16[bayerIndex] % 4, kBayerMatrix16[bayerIndex] / 4);//写进1/4RT的那个像素在4*4块内的位置
        ivec2 workDeltaPos = workPos % 4;//该像素在 4×4 块内的位置
        const bool bUpdateEvaluate = (workDeltaPos.x == bayerOffset.x) && (workDeltaPos.y == bayerOffset.y);//本帧 raymarching 有没有算过当前这个全分辨率像素
        if(bUpdateEvaluate)//这个全分辨率像素，正好是本帧 raymarching 写进 1/4 RT 的那一个
        {
            depthZ = curDepthZ;
        #if 0//已修复：原先这里恒为1，永远走"直接用本帧原始噪点"这条路，下面带方差钳制的
             //真正时域混合分支（#else）成了死代码——这正是高速飞行时稀疏 raymarch 噪点
             //花屏、怎么飞怎么不收敛的核心原因之一（另一半是 camViewProjPrev 的修复，
             //见 fillCloudFrameData()）。下面 fog 一直就是这套"方差钳制混合"在跑，
             //这里改成和 fog 同样的路径，不是新写逻辑。
            // Just update color is good enough.
            color = curColor;
        #else//执行：与下方 fog 分支同款方差钳制时域混合
            if(abs(preDepthZ - curDepthZ) > 0.1)
            {
                color = curColor;
            }
            else
            {
                vec4 preColor = texture(sampler2D(inCloudReconstructionTextureHistory,  linearClampEdgeSampler), uvPrev); // catmullRom9Sample(inCloudReconstructionTextureHistory, linearClampEdgeSampler, uvPrev, vec2(textureSize(inCloudReconstructionTextureHistory, 0)));

                // variance clamp.
                vec4 clampColorHistory;
                {
                    float wsum = 0.0f;
                    vec4 vsum  = vec4(0.0f);
                    vec4 vsum2 = vec4(0.0f);

                    for (int y = -1; y <= 1; ++y)
                    {
                        for (int x = -1; x <= 1; ++x)
                        {
                            const vec4 neigh = texture(sampler2D(inCloudRenderTexture, pointClampEdgeSampler), uv + curEvaluateCloudTexelSize * ivec2(x, y));
                            const float w = exp(-0.75f * (x * x + y * y));
                            vsum2 += neigh * neigh * w;
                            vsum  += neigh * w;
                            wsum  += w;
                        }
                    }

                    const vec4 ex  = vsum / wsum;
                    const vec4 ex2 = vsum2 / wsum;
                    const vec4 dev = sqrt(max(ex2 - ex * ex, 0.0f));

                    const float boxSize = 2.5f;

                    vec4 nmin = ex - dev * boxSize;
                    vec4 nmax = ex + dev * boxSize;

                    clampColorHistory = clamp(preColor, nmin, nmax);
                }

                color  = mix(clampColorHistory,  curColor, 0.5);
            }
        #endif

        #if 0//#if 0的代码不执行
            fog = curFog;
        #else//执行
            if(abs(preDepthZ - curDepthZ) > 0.1)//云的深度不一致（云层遮挡关系变了/云位置变了/相机位置变了）,0.1阈值为经验值
            {
                fog = curFog;//只用本帧curFog
            }
            else//深度一致
            {
                vec4 preColor = texture(sampler2D(inCloudFogReconstructionTextureHistory,  linearClampEdgeSampler), uvPrev); //上一帧全分辨率雾 history，按 uvPrev 重投影采样

                // variance clamp.
                vec4 clampColorHistory;
                {
                    float wsum = 0.0f;
                    vec4 vsum  = vec4(0.0f);
                    vec4 vsum2 = vec4(0.0f);

                    for (int y = -1; y <= 1; ++y)
                    {
                        for (int x = -1; x <= 1; ++x)//当前1/4雾RT的采样像素周围取3*3邻域
                        {
                            const vec4 neigh = texture(sampler2D(inCloudFogRenderTexture, pointClampEdgeSampler), uv + curEvaluateCloudTexelSize * ivec2(x, y));//本帧 1/4 雾在周围 texel 的值（当前帧邻域，作参考）
                            const float w = exp(-0.75f * (x * x + y * y));// 中心权重大，角落权重小
                            vsum2 += neigh * neigh * w;//加权平方和
                            vsum  += neigh * w;
                            wsum  += w;//权重和
                        }
                    }

                    const vec4 ex  = vsum / wsum;//加权均值
                    const vec4 ex2 = vsum2 / wsum;//加权平方均值
                    const vec4 dev = sqrt(max(ex2 - ex * ex, 0.0f));//标准差σ

                    const float boxSize = 2.5f;//TAA系数，越大 history 允许偏离越多

                    vec4 nmin = ex - dev * boxSize;
                    vec4 nmax = ex + dev * boxSize;

                    clampColorHistory = clamp(preColor, nmin, nmax);//把 history 限制在「本帧邻域 ± 2.5σ」内，去掉离谱拖影
                }

                fog  = mix(clampColorHistory,  curFog, 0.5);//各50%混合history和curFog，不仅稳定又要能跟上变化
            }
        #endif
        }
        else//上一帧 history 可信，但本帧 raymarching 没算这个像素；raymarching只算4×4块里1个像素，其余15个走这里；其余15个像素直接从上一帧全分辨率history按重投影uv拷贝云数据
        {
            // Prev uv valid, sample history with prev Uv.
            color  =  texture(sampler2D(inCloudReconstructionTextureHistory,  linearClampEdgeSampler), uvPrev);//读上一帧全分辨率云颜色 history，RGB = 散射L，A = 透射T
            fog  =  texture(sampler2D(inCloudFogReconstructionTextureHistory,  linearClampEdgeSampler), uvPrev);//上一帧全分辨率雾/God Ray history
            depthZ = preDepthZ;//上一帧的代表深度
            //linearClampEdgeSampler：在 history 上双线性采样；边缘 clamp，避免 repeat 伪影
        }
    }
    else // bPrevUvValid == false：history 不可用时的 fallback
    {
        // 触发条件（见 39 行 bPrevUvValid）：
        //   • uvPrev 飞出 [0,1]（云重投影到屏幕外，例如相机大幅转向）
        //   • bCameraCut == true（切镜头/传送，上一帧与当前帧对不上）
        //
        // 无法做时域重投影 / history 填洞，只能把 1/4 RT 双线性上采样到全分辨率。
        // 画质比 Bayer+history 路径略糊，但不会出现错误的历史拖影。
        // 首帧、无 history RT、或 RS3D 初版未接 ping-pong 时也会长期走此分支。
        color  = texture(sampler2D(inCloudRenderTexture, linearClampEdgeSampler), uv);
        // 云 L+T：RGB=散射光，A=视线透射；uv 为全分辨率像素中心，从 1/4 inCloudRenderTexture 插值

        fog = texture(sampler2D(inCloudFogRenderTexture, linearClampEdgeSampler), uv);
        // God Ray / 低层体积雾；RS3D 若未启用 God Ray，此 RT 可能全 0

        depthZ = texture(sampler2D(inCloudDepthTexture, linearClampEdgeSampler), uv).r;
        // 云代表深度 cloudZ；与 texelFetch 点采样不同，这里对 1/4 深度做双线性插值
    }

    // ── 写入全分辨率重建 RT（供 cloud_composite 与下一帧 history ping-pong）──
    imageStore(imageCloudReconstructionTexture, workPos, color);
    // binding 12：全分辨率云颜色；flower 约定 RGB=散射 L，A=透射 T（非 premultiplied）

    imageStore(imageCloudFogReconstructionTexture, workPos, fog);
    // binding 24：全分辨率雾/God Ray；composite pass 叠加到场景

    imageStore(imageCloudDepthReconstructionTexture, workPos, vec4(depthZ));
    // binding 16：全分辨率云深度；下一帧作为 inCloudDepthReconstructionTextureHistory 参与重投影



}