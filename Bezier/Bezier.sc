Bezier : UGen {
	*ar { |res = 1024, ctrlX = 512, ctrlY = 0.5|
		/* TODO */
		^this.multiNew('audio', res, ctrlX, ctrlY);
	}
    
	checkInputs {
		/* TODO */
		^this.checkValidInputs;
	}
}
