\documentclass{article}
\usepackage{longtable}
\usepackage[english]{babel}
\begin{document}
\section{Statistics}
\begin{center}
    \begin{longtable}{| l | l | l | l | l | p{5cm} |}
    \caption[Statistics for all taxels, ordered by $\sigma$]{Statistics for all taxels, ordered by $\sigma$} \label{grid_mlmmh} \\

    \multicolumn{1}{|c|}{\textbf{$\sigma$}} & \multicolumn{1}{c|}{\textbf{$\mu$}} & \multicolumn{1}{c|}{\textbf{$\mu_{1/2}$}} & \multicolumn{1}{c|}{\textbf{cdc}} & \multicolumn{1}{c|}{\textbf{taxel}}\\ \hline 
    \endfirsthead

    \multicolumn{5}{c}%
    {{\bfseries \tablename\ \thetable{} -- continued from previous page}} \\
    \multicolumn{1}{|c|}{\textbf{sigma}} & \multicolumn{1}{c|}{\textbf{mu}} & \multicolumn{1}{c|}{\textbf{median}} & \multicolumn{1}{c|}{\textbf{cdc}} & \multicolumn{1}{c|}{\textbf{taxel}}\\ \hline 
    \endhead

    \hline \multicolumn{5}{|r|}{{Continued on next page}} \\ \hline
    \endfoot

    \hline \hline
    \endlastfoot
    
    ${for v in values:}$${'%f'%v['sigma']}$ & ${'%f'%v['mu']}$ & ${'%d'%v['median']}$ & ${'%d'%v['cdc']}$ & ${'%d'%v['taxel']}$\\
    ${:end-for}$
    \end{longtable}
\end{center}
\end{document}