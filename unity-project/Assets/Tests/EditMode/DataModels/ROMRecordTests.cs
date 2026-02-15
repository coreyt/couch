using NUnit.Framework;
using AnkleSim.Core.DataModels;

namespace AnkleSim.Tests.EditMode.DataModels
{
    public class ROMRecordTests
    {
        [Test]
        public void NewRecord_DefaultsAreZero()
        {
            var record = new ROMRecord();

            Assert.AreEqual(0f, record.maxDorsiflexion);
            Assert.AreEqual(0f, record.maxPlantarflexion);
            Assert.AreEqual(0f, record.totalSagittalArc);
            Assert.AreEqual(0f, record.inversion);
            Assert.AreEqual(0f, record.eversion);
            Assert.AreEqual(0f, record.dorsiflexionTorque);
            Assert.AreEqual(0f, record.plantarflexionTorque);
            Assert.AreEqual(0d, record.timestamp);
        }

        [Test]
        public void RecalculateTotalArc_SumsDFAndPF()
        {
            var record = new ROMRecord
            {
                maxDorsiflexion = 15f,
                maxPlantarflexion = 25f
            };

            record.RecalculateTotalArc();

            Assert.AreEqual(40f, record.totalSagittalArc, 0.001f);
        }

        [Test]
        public void RecalculateTotalArc_WithZeroValues_ReturnsZero()
        {
            var record = new ROMRecord();
            record.RecalculateTotalArc();
            Assert.AreEqual(0f, record.totalSagittalArc, 0.001f);
        }
    }
}
