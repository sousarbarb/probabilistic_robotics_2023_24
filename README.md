# [probabilistic_robotics_2023_24](https://github.com/sousarbarb/probabilistic_robotics_2023_24)

Remote Lectures relative to Probabilistic Robotics 2023/2024 @ Sapienza
Università di Roma. The official website for the curricular unit is
https://sites.google.com/diag.uniroma1.it/probabilistic-robotics-2023-24,
and all the source code and lecture materials are available in a GitLab
repository
[probabilistic_robotics_2023_24](https://gitlab.com/grisetti/probabilistic_robotics_2023_24).
For accessing the recorded videos from the lectures, the ones available are
relative to the academic year 2020/2021
([probabilistic_robotics_2020_21/lectures](https://sites.google.com/diag.uniroma1.it/probabilistic-robotics-2020-21/lectures)) and 2021/22
([probabilistic_robotics_2021_22/lectures](https://sites.google.com/diag.uniroma1.it/probabilistic-robotics-2021-22/lectures)).

## Setup

### LaTeX

```sh
sudo apt update
sudo apt install texlive-full
tex --version
latex --version
pdflatex --version
```

TBC

### Octave

```sh
sudo apt update
sudo apt install octave liboctave-dev
```

## Usage

### LaTeX

**Build**

```sh
cd <project dir>
latexmk -bibtex -pdf    # First time to generate bibliography references
latexmk -bibtex -pdf    # Final PDF (when using .bib files...)
```

**Clean**

```sh
latexmk -C
```

See
[latexmk documentation](https://ftp.eq.uc.pt/software/TeX/support/latexmk/latexmk.pdf)
for more information.

### Octave

```sh

```

## Acknowledgements

A special thanks to Prof. Dr. Giorgio Grisetti
([@grisetti](https://gitlab.com/grisetti),
[website](https://sites.google.com/dis.uniroma1.it/grisetti/home)) for making
available all the materials relative to the course Probabilistic Robotics at
[Sapienza Università di Roma](https://www.uniroma1.it/en/pagina-strutturale/home).

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.inesctec.pt/ricardo.b.sousa),
  [mail:inesctec](mailto:ricardo.b.sousa@inesctec.pt),
  [mail:personal](mailto:sousa.ricardob@outlook.com),
  [mail:professor](mailto:rbs@fe.up.pt),
  [mail:student](mailto:up201503004@edu.fe.up.pt))
