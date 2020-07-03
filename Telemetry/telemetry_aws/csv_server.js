const app 		      = require('express')();
const path          = require('path');
const bodyparser 	  = require('body-parser');
const bb         	  = require('express-busboy');
const child_process = require('child_process');
const rimraf        = require('rimraf');
const PORT          = 5000;

app.use(bodyparser.urlencoded({ extended: true }));

bb.extend(app, {
    upload: true,
    path: 'uploads',
    allowedPath: /./
});

function clean (req) {
  return rimraf(
    `./uploads/${req.files.csv.uuid}`,
    err => console.log((err ? "Error deleting" : "Deleted"), req.files.csv.filename)
  );
}

app.get('/', (req, res) => res.sendFile(path.resolve(__dirname, 'index.html')));
app.post('/upload', (req, res) => {
	const db_name = req.body.db_name.trim().replace(/ /g,"_");
	if (!db_name) return res.redirect('/?err=true');

	child_process.exec(`python3 csv_to_influx.py ${db_name} ./${req.files.csv.file}`, () => {}) //clean(req))
	res.send(`Your CSV is being written to database ${db_name}. Go to <a href=http://ec2-3-134-2-166.us-east-2.compute.amazonaws.com:3000/>Grafana</a> to view`);
});

app.listen(PORT, () => console.log("Grafana upload server running on port " + PORT));
