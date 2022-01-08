import 'package:flutter/material.dart';

import '../block_view.dart';

class SelectableBlockWidget extends StatelessWidget {
  const SelectableBlockWidget(
      {required this.setBlock,
      required this.blockToRender,
      Key? key,
      this.hint = 'Hold to select a block',
      this.desiredTypes})
      : super(key: key);

  final Function(Block?) setBlock;
  final Block? blockToRender;
  final String hint;
  final List<BlockType>? desiredTypes;

  @override
  Widget build(BuildContext context) {
    return InkWell(
      child: blockToRender != null
          ? blockToRender!.construct()
          : Card(
              color: Colors.grey,
              shape: ContinuousRectangleBorder(
                side: const BorderSide(color: Colors.black45, width: 1),
                borderRadius: BorderRadius.circular(15),
              ),
              child: Padding(
                padding: const EdgeInsets.all(10),
                child: Text(hint, style: const TextStyle(color: Colors.white)),
              )),
      onLongPress: () {
        desiredTypes == null
            ? selectBlockFromList(context).then((Block? block) {
                setBlock(block);
              })
            : selectBlockFromList(context, desiredTypes: desiredTypes!)
                .then((Block? block) {
                setBlock(block);
              });
      },
    );
  }
}
