//
//  ContentView.swift
//  metal-explore
//
//  Created by Umercantcode on 07/01/2026.
//

import SwiftUI
import SwiftData

struct ContentView: View {
    @Environment(\.modelContext) private var modelContext
    @Query private var items: [Item]

    var body: some View {
        MetalView()
//    .ignoresSafeArea()
    }


}

#Preview {
    ContentView()
        .modelContainer(for: Item.self, inMemory: true)
}
