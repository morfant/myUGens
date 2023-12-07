Bezier : UGen {
	*ar { |freq = 440.0, ctrlX = 0.5, mul = 1.0, add = 0.0|
		/* TODO */
		^this.multiNew('audio', freq, ctrlX).madd(mul, add);
	}
    
	checkInputs {
		/* TODO */
		^this.checkValidInputs;
	}
}
