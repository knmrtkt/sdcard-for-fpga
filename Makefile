VIVADO = /opt/Xilinx/Vivado/2019.2/bin/vivado

bit:
	${VIVADO} -mode batch -source build.tcl
	mv project.rpt vivado
	cp vivado/vivado.runs/impl_1/*.bit .

clean:
	rm -f *.log *.jou
	rm -rf vivado/vivado.*
	rm -rf vivado/ip/*
