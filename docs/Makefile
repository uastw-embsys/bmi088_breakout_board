all : bmi088_breakout_I2C.svg bmi088_breakout_SPI.svg

bmi088_breakout_I2C.svg : bmi088_breakout_I2C.pdf
	inkscape bmi088_breakout_I2C.pdf --export-filename bmi088_breakout_I2C.svg --export-background=white
	
bmi088_breakout_SPI.svg : bmi088_breakout_SPI.pdf
	inkscape bmi088_breakout_SPI.pdf --export-filename bmi088_breakout_SPI.svg --export-background=white
	
bmi088_breakout_I2C.pdf :  bmi088_breakout_I2C.tex
	pdflatex  bmi088_breakout_I2C
	
bmi088_breakout_SPI.pdf :  bmi088_breakout_SPI.tex
	pdflatex  bmi088_breakout_SPI

