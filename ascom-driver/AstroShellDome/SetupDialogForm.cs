using System;
using System.Drawing;
using System.Windows.Forms;

namespace ASCOM.AstroShellDome
{
    public partial class SetupDialogForm : Form
    {
        public string ArduinoIP
        {
            get { return textBoxIP.Text.Trim(); }
            set { textBoxIP.Text = value; }
        }

        public int ArduinoPort
        {
            get
            {
                int port;
                return int.TryParse(textBoxPort.Text.Trim(), out port) ? port : 80;
            }
            set { textBoxPort.Text = value.ToString(); }
        }

        public bool SwapOpenClose
        {
            get { return checkBoxSwap.Checked; }
            set { checkBoxSwap.Checked = value; }
        }

        public SetupDialogForm()
        {
            InitializeComponent();
        }

        private void buttonTest_Click(object sender, EventArgs e)
        {
            labelTestResult.Text = "Testing...";
            labelTestResult.ForeColor = Color.Gray;
            labelTestResult.Refresh();

            try
            {
                using (var client = new ArduinoClient(ArduinoIP, ArduinoPort, 5))
                {
                    string status = client.GetSimpleStatus();
                    labelTestResult.Text = $"Connected! Dome is: {status}";
                    labelTestResult.ForeColor = Color.Green;
                }
            }
            catch (Exception ex)
            {
                string msg = ex.InnerException != null ? ex.InnerException.Message : ex.Message;
                labelTestResult.Text = $"Failed: {msg}";
                labelTestResult.ForeColor = Color.Red;
            }
        }
    }
}
