TEX     := tail_rotor_dynamics.tex
PDF     := $(TEX:.tex=.pdf)
VIEWER  ?= xdg-open

APT_PKGS := texlive-latex-recommended texlive-latex-extra texlive-science texlive-fonts-recommended latexmk

.PHONY: all pdf view install clean distclean

all: pdf

pdf: $(PDF)

$(PDF): $(TEX)
	latexmk -pdf -interaction=nonstopmode -halt-on-error $<

view:
	$(VIEWER) $(PDF) &

install:
	sudo apt-get install -y $(APT_PKGS)

clean:
	latexmk -c $(TEX)

distclean:
	latexmk -C $(TEX)
