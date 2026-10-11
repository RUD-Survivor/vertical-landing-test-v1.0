layout(push_constant) uniform PC {
    mat4  invViewProj;       // 64
    float skyVibrancy;       //  4 — offset 64
    float exposure;          //  4
    float starDensity;       //  4
    float starBrightness;    //  4
    float starSize;          //  4
    float glowStrength;      //  4
    float milkyWayIntensity; //  4
    float milkyWayWidth;     //  4
    float dustStrength;      //  4
    float galacticCoreBoost; //  4
    float colorVariety;      //  4
    float bgLevel;           //  4
    float galaxyYaw;         //  4
    float galaxyPitch;       //  4
    float _pad0;
    float _pad1;
} pc;
