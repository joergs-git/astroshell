namespace ASCOM.AstroShellDome
{
    partial class SetupDialogForm
    {
        private System.ComponentModel.IContainer components = null;

        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        private void InitializeComponent()
        {
            this.labelIP = new System.Windows.Forms.Label();
            this.textBoxIP = new System.Windows.Forms.TextBox();
            this.labelPort = new System.Windows.Forms.Label();
            this.textBoxPort = new System.Windows.Forms.TextBox();
            this.checkBoxSwap = new System.Windows.Forms.CheckBox();
            this.buttonTest = new System.Windows.Forms.Button();
            this.labelTestResult = new System.Windows.Forms.Label();
            this.buttonOK = new System.Windows.Forms.Button();
            this.buttonCancel = new System.Windows.Forms.Button();
            this.groupBoxConnection = new System.Windows.Forms.GroupBox();
            this.groupBoxOptions = new System.Windows.Forms.GroupBox();

            this.groupBoxConnection.SuspendLayout();
            this.groupBoxOptions.SuspendLayout();
            this.SuspendLayout();

            // labelIP
            this.labelIP.AutoSize = true;
            this.labelIP.Location = new System.Drawing.Point(15, 30);
            this.labelIP.Name = "labelIP";
            this.labelIP.Size = new System.Drawing.Size(70, 13);
            this.labelIP.Text = "Arduino IP:";

            // textBoxIP
            this.textBoxIP.Location = new System.Drawing.Point(120, 27);
            this.textBoxIP.Name = "textBoxIP";
            this.textBoxIP.Size = new System.Drawing.Size(150, 20);

            // labelPort
            this.labelPort.AutoSize = true;
            this.labelPort.Location = new System.Drawing.Point(15, 60);
            this.labelPort.Name = "labelPort";
            this.labelPort.Size = new System.Drawing.Size(70, 13);
            this.labelPort.Text = "Port:";

            // textBoxPort
            this.textBoxPort.Location = new System.Drawing.Point(120, 57);
            this.textBoxPort.Name = "textBoxPort";
            this.textBoxPort.Size = new System.Drawing.Size(60, 20);

            // buttonTest
            this.buttonTest.Location = new System.Drawing.Point(120, 90);
            this.buttonTest.Name = "buttonTest";
            this.buttonTest.Size = new System.Drawing.Size(110, 25);
            this.buttonTest.Text = "Test Connection";
            this.buttonTest.UseVisualStyleBackColor = true;
            this.buttonTest.Click += new System.EventHandler(this.buttonTest_Click);

            // labelTestResult
            this.labelTestResult.AutoSize = true;
            this.labelTestResult.Location = new System.Drawing.Point(15, 125);
            this.labelTestResult.Name = "labelTestResult";
            this.labelTestResult.Size = new System.Drawing.Size(250, 13);
            this.labelTestResult.ForeColor = System.Drawing.Color.Gray;
            this.labelTestResult.Text = "";

            // groupBoxConnection
            this.groupBoxConnection.Controls.Add(this.labelIP);
            this.groupBoxConnection.Controls.Add(this.textBoxIP);
            this.groupBoxConnection.Controls.Add(this.labelPort);
            this.groupBoxConnection.Controls.Add(this.textBoxPort);
            this.groupBoxConnection.Controls.Add(this.buttonTest);
            this.groupBoxConnection.Controls.Add(this.labelTestResult);
            this.groupBoxConnection.Location = new System.Drawing.Point(12, 12);
            this.groupBoxConnection.Name = "groupBoxConnection";
            this.groupBoxConnection.Size = new System.Drawing.Size(290, 150);
            this.groupBoxConnection.Text = "Connection";

            // checkBoxSwap
            this.checkBoxSwap.AutoSize = true;
            this.checkBoxSwap.Location = new System.Drawing.Point(15, 25);
            this.checkBoxSwap.Name = "checkBoxSwap";
            this.checkBoxSwap.Size = new System.Drawing.Size(250, 17);
            this.checkBoxSwap.Text = "Swap Open/Close commands";

            // groupBoxOptions
            this.groupBoxOptions.Controls.Add(this.checkBoxSwap);
            this.groupBoxOptions.Location = new System.Drawing.Point(12, 170);
            this.groupBoxOptions.Name = "groupBoxOptions";
            this.groupBoxOptions.Size = new System.Drawing.Size(290, 55);
            this.groupBoxOptions.Text = "Options";

            // buttonOK
            this.buttonOK.DialogResult = System.Windows.Forms.DialogResult.OK;
            this.buttonOK.Location = new System.Drawing.Point(146, 238);
            this.buttonOK.Name = "buttonOK";
            this.buttonOK.Size = new System.Drawing.Size(75, 25);
            this.buttonOK.Text = "OK";
            this.buttonOK.UseVisualStyleBackColor = true;

            // buttonCancel
            this.buttonCancel.DialogResult = System.Windows.Forms.DialogResult.Cancel;
            this.buttonCancel.Location = new System.Drawing.Point(227, 238);
            this.buttonCancel.Name = "buttonCancel";
            this.buttonCancel.Size = new System.Drawing.Size(75, 25);
            this.buttonCancel.Text = "Cancel";
            this.buttonCancel.UseVisualStyleBackColor = true;

            // SetupDialogForm
            this.AcceptButton = this.buttonOK;
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.CancelButton = this.buttonCancel;
            this.ClientSize = new System.Drawing.Size(314, 275);
            this.Controls.Add(this.groupBoxConnection);
            this.Controls.Add(this.groupBoxOptions);
            this.Controls.Add(this.buttonOK);
            this.Controls.Add(this.buttonCancel);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.Name = "SetupDialogForm";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "AstroShell Dome Setup";

            this.groupBoxConnection.ResumeLayout(false);
            this.groupBoxConnection.PerformLayout();
            this.groupBoxOptions.ResumeLayout(false);
            this.groupBoxOptions.PerformLayout();
            this.ResumeLayout(false);
        }

        #endregion

        private System.Windows.Forms.Label labelIP;
        private System.Windows.Forms.TextBox textBoxIP;
        private System.Windows.Forms.Label labelPort;
        private System.Windows.Forms.TextBox textBoxPort;
        private System.Windows.Forms.CheckBox checkBoxSwap;
        private System.Windows.Forms.Button buttonTest;
        private System.Windows.Forms.Label labelTestResult;
        private System.Windows.Forms.Button buttonOK;
        private System.Windows.Forms.Button buttonCancel;
        private System.Windows.Forms.GroupBox groupBoxConnection;
        private System.Windows.Forms.GroupBox groupBoxOptions;
    }
}
