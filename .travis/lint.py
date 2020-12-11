import anybadge
from pylint.lint import Run

results = Run(['fleet_management'], do_exit=False)
score = results.linter.stats['global_note']
thresholds = {2: 'red',
              4: 'orange',
              6: 'yellow',
              10: 'green'}

badge = anybadge.Badge('pylint score', round(score, 2), thresholds=thresholds)

badge.write_badge('pylint.svg')
