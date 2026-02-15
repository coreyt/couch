using NUnit.Framework;
using AnkleSim.Core.DataModels;

namespace AnkleSim.Tests.EditMode.DataModels
{
    public class ResectionRecordTests
    {
        [Test]
        public void StoresCorrectVolume()
        {
            var record = new ResectionRecord
            {
                targetBone = BoneType.Tibia,
                resectionDepth = 8f,
                volumeRemoved = 1250.5f
            };

            Assert.AreEqual(BoneType.Tibia, record.targetBone);
            Assert.AreEqual(1250.5f, record.volumeRemoved, 0.001f);
            Assert.AreEqual(8f, record.resectionDepth, 0.001f);
        }
    }
}
