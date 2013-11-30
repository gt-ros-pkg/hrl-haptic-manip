# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Library General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor Boston, MA 02110-1301,  USA

from templite import Templite
import subprocess

class StatisticsTable:
    def __init__(self, stats, name):
        f = file('table.tpl', 'r')
        template_table = f.read()
        f.close()

        self.template = Templite(template_table)
        self.stats = stats
        self.name = name
        
    def render(self):
        return self.template.render(values = self.stats)
        
    def render_pdf(self):
        process = subprocess.Popen(['pdflatex', '-jobname=%s'%self.name], shell=False, stdin=subprocess.PIPE)
        process.communicate(self.render())