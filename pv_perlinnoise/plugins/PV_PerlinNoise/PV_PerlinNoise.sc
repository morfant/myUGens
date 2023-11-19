PV_PerlinNoise : PV_ChainUGen {
    *new { arg buffer, magScale = 1.0, stepScale = 0.02, octave = 1.0, persistence = 0.8, seed = 0;
        if (stepScale >= 1.0, {
            "stepScale has to be less than 1, lager than 0".error;
            stepScale = 0.99;
        });
        if (stepScale <= 0, {
            "stepScale has to be less than 1, lager than 0".error;
            stepScale = 0.0001;
        });
        
        if (octave >= 8, {
            "octave is limited in range 1 ~ 8".error;
            octave = 8;
        });
        if (octave < 1, {
            "octave is limited in range 1 ~ 8".error;
            octave = 1;
        });
        
        if (persistence >= 1.0, {
            "persistence is limited in range 0.01 ~ 1.0".error;
            persistence = 1.0;
        });
        if (persistence <= 0.0, {
            "persistence is limited in range 0.01 ~ 1.0".error;
            persistence = 0.001;
        });
        
        ^this.multiNew('control', buffer, magScale, stepScale, octave, persistence, seed)
    }
}
